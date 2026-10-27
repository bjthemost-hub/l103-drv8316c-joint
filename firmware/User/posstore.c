#include "posstore.h"
#include "board.h"
#include "debug.h"
#include "fmt.h"
#include "enc.h"
#include "motion.h"
#include "ch32l103_flash.h"
#include "drv8316c.h"      /* Micros() */

/* =============================================================================
 * 位置 / 机械零点的掉电保存 —— 说明见 posstore.h
 * ========================================================================== */

#define POS_BASE_A        0x0800FD00u
#define POS_BASE_B        0x0800FE00u
#define POS_REC_WORDS     16u
#define POS_RECS          4u
#define POS_MAGIC         0x56503231u        /* 'VP21' */

#define POS_SAVE_STILL_US 3000000u           /* 完全不动 3 s 才写 */
#define POS_STILL_VEL     0.05f              /* rad/s, ≈0.5 rpm, 低于它算停住 */
#define POS_CHG_RAD       0.010f             /* 位置变化 < 0.57° 就不重复写, 省擦写次数 */
#define POS_STILL_WIN_RAD 0.005f             /* 3 s 窗口内净位移 < 0.29° 算"完全停住" */
#define POS_TWO_PI        6.28318530718f

static uint32_t shadow[64];
static int      inited;
static int      usable;
static int      restored;      /* 上电从 Flash 里读到了记录 */
static int      home_loaded;                 /* 记录里的"设过机械零位" */
static float    saved_mech;                  /* 记录里的机械坐标 rad */
static uint32_t last_seq;
static uint32_t cur_seq;
static uint32_t t_mark;                      /* 最后一次"还在动/还在忙"的时刻 */
static float    pos_mark;                    /* 那时刻的机械坐标 (判净位移用) */
static uint8_t  pending;                     /* 有活要写 */
static uint8_t  urgent;                      /* 零位被显式改过: 不用等 3 s */
static uint32_t wbase;                       /* 下一次写到哪一页 */
static int      wslot;                       /* 下一次写到哪个槽 */
static uint32_t n_ok, n_fail;

static uint32_t flash_off(uint32_t a) { return a & 0x00FFFFFFu; }

/* 程序镜像末尾 (跟 nvcfg.c 同一口径): 顶到记录区就停用, 免得把自己擦掉 */
static uint32_t code_flash_end(void)
{
    extern char _data_lma[];
    extern char _data_vma[];
    extern char _edata[];

    return (uint32_t)_data_lma + (uint32_t)(_edata - _data_vma);
}

static uint32_t rd32(uint32_t a) { return *(volatile uint32_t *)a; }

static void page_read(uint32_t base)
{
    uint32_t i;
    for(i = 0; i < 64u; i++) shadow[i] = rd32(base + 4u * i);
}

static void page_program(uint32_t base)
{
    uint32_t i;

    FLASH_Unlock_Fast();
    FLASH_BufReset();
    for(i = 0; i < 64u; i++) FLASH_BufLoad(base + 4u * i, shadow[i]);
    FLASH_ProgramPage_Fast(base);
    FLASH_Lock_Fast();
}

static void page_erase(uint32_t base)
{
    uint32_t i;

    FLASH_Unlock_Fast();
    FLASH_ErasePage_Fast(base);
    FLASH_Lock_Fast();
    for(i = 0; i < 64u; i++) shadow[i] = 0xFFFFFFFFu;
}

static uint32_t *rec(int slot)
{
    return &shadow[(uint32_t)slot * POS_REC_WORDS];
}

/* FNV-1a over w0..w4 —— 位翻转/半截写能查出来 */
static uint32_t rec_crc(const uint32_t *w)
{
    uint32_t h = 2166136261u;
    int i, b;

    for(i = 0; i < 5; i++)
        for(b = 0; b < 4; b++)
        {
            h ^= (uint32_t)((w[i] >> (8 * b)) & 0xFFu);
            h *= 16777619u;
        }
    return h;
}

static int rec_valid(int slot)
{
    const uint32_t *r = rec(slot);
    if(r[0] != POS_MAGIC) return 0;
    return (r[5] == rec_crc(r)) ? 1 : 0;
}

static uint32_t other_page(uint32_t base)
{
    return (base == POS_BASE_A) ? POS_BASE_B : POS_BASE_A;
}

/* 两页里取 seq 最大的那条; 1 = 有 */
static int scan(uint32_t *pbase, int *pslot)
{
    uint32_t bases[2];
    uint32_t best = 0u;
    int i, s, found = 0;

    bases[0] = POS_BASE_A;
    bases[1] = POS_BASE_B;

    for(i = 0; i < 2; i++)
    {
        page_read(bases[i]);
        for(s = 0; s < (int)POS_RECS; s++)
        {
            if(!rec_valid(s)) continue;
            if(!found || rec(s)[1] > best)
            {
                found = 1;
                best  = rec(s)[1];
                *pbase = bases[i];
                *pslot = s;
            }
        }
    }
    return found;
}

int      PosStore_Usable(void)     { return usable; }
int      PosStore_Restored(void)   { return restored; }
int      PosStore_HomeSet(void)    { return home_loaded; }
float    PosStore_SavedMech(void)  { return saved_mech; }
uint32_t PosStore_Seq(void)        { return last_seq; }
uint32_t PosStore_WriteOk(void)    { return n_ok; }
uint32_t PosStore_WriteFail(void)  { return n_fail; }

void PosStore_Init(void)
{
    uint32_t base = POS_BASE_A;
    int      slot = 0;

    inited   = 1;
    usable   = 0;
    restored = 0;
    home_loaded = 0;
    saved_mech  = 0.0f;
    last_seq = cur_seq = 0u;
    pending = urgent = 0u;
    t_mark = 0u;
    pos_mark = 0.0f;              /* 末尾会按实际位置重设, 见 Init 结尾 */
    wbase  = POS_BASE_A;
    wslot  = 0;
    n_ok = n_fail = 0u;

    if(flash_off(code_flash_end()) >= flash_off(POS_BASE_A))
    {
        printf("POS: DISABLED - code end 0x%06lX reached 0x%06lX\r\n",
               (unsigned long)flash_off(code_flash_end()),
               (unsigned long)flash_off(POS_BASE_A));
        return;
    }
    usable = 1;

    if(!scan(&base, &slot))
    {
        pos_mark = Enc_MechRad();
        printf("POS: 无记录 (设零点 / 停稳 3s 后自动写)\r\n");
        return;
    }

    {
        const uint32_t *r;
        float    mech, zero, zero_use, x;
        int32_t  rev;
        uint32_t fl;

        page_read(base);
        r   = rec(slot);
        mech = (float)(int32_t)r[2] * 0.0001f;
        zero = (float)(int32_t)r[3] * 0.0001f;
        fl   = r[4];

        restored    = 1;
        home_loaded = (fl & 1u) ? 1 : 0;
        saved_mech  = mech;
        last_seq    = r[1];
        cur_seq     = last_seq + 1u;

        /* 单圈绝对: 只要断电期间轴转得 < 180°, rev 就能唯一确定。
         * 本产品最多 720°, 这条前提成立。 */
        zero_use = home_loaded ? zero : 0.0f;
        x = (mech + zero_use) / POS_TWO_PI - Enc_TurnPos();
        rev = (int32_t)(x >= 0.0f ? (x + 0.5f) : (x - 0.5f));

        Enc_SetRev(rev);
        Enc_SetZeroRad(zero_use);

        printf("POS: 恢复 seq=%lu mech=%s rad rev=%ld home=%u\r\n",
               (unsigned long)last_seq, FmtF(mech, 3), (long)rev, (unsigned)home_loaded);
    }

    wbase = base;
    wslot = slot + 1;
    if(wslot >= (int)POS_RECS) { wbase = other_page(base); wslot = 0; }
    else
    {
        page_read(wbase);
        if(rec(wslot)[0] != 0xFFFFFFFFu) { wbase = other_page(base); wslot = 0; }
    }

    /* 上电这一下不算"动过": 否则每次开机都要白写一次 Flash, 白白消耗擦写次数 */
    pos_mark = Enc_MechRad();
    pending  = 0u;
    urgent   = 0u;
}

void PosStore_RequestSave(void)
{
    pending = 1u;
    urgent  = 1u;
    t_mark  = Micros();
}

static int changed(void)
{
    float m = Motion_PosMechRad();
    float d = m - saved_mech;

    if(Motion_HomeSet() != home_loaded) return 1;
    if(d > POS_CHG_RAD || d < -POS_CHG_RAD) return 1;
    return 0;
}

static int do_save(void)
{
    float    mech = Motion_PosMechRad();
    float    zero = Enc_ZeroRad();
    int      home = Motion_HomeSet();
    uint32_t seq, *r;
    int      i;

    if(!usable) return 0;

    page_read(wbase);

    /* 本页写满 / 槽里不是空白 -> 换另一页, 擦干净再写。
     * 顺序保证: 擦的是**旧的那页**, 最新记录还在另一页上。 */
    if(wslot >= (int)POS_RECS || rec(wslot)[0] != 0xFFFFFFFFu)
    {
        wbase = other_page(wbase);
        page_erase(wbase);
        wslot = 0;
    }

    r   = rec(wslot);
    seq = cur_seq++;

    r[0] = POS_MAGIC;
    r[1] = seq;
    r[2] = (uint32_t)(int32_t)(mech * 10000.0f + (mech >= 0.0f ? 0.5f : -0.5f));
    r[3] = (uint32_t)(int32_t)(zero * 10000.0f + (zero >= 0.0f ? 0.5f : -0.5f));
    r[4] = (uint32_t)(home ? 1u : 0u);
    r[5] = rec_crc(r);
    for(i = 6; i < (int)POS_REC_WORDS; i++) r[i] = 0u;

    page_program(wbase);

    /* 回读核对: 写没写进去要能被看见 */
    page_read(wbase);
    if(rec_valid(wslot) && rec(wslot)[1] == seq)
    {
        saved_mech  = mech;
        home_loaded = home;
        last_seq    = seq;
        n_ok++;
        wslot++;
        return 1;
    }

    n_fail++;
    /* 写失败: 下一笔换页重来, 不在原地打转 */
    wbase = other_page(wbase);
    wslot = 0;
    return 0;
}

void PosStore_Tick(uint32_t now_us)
{
    float mech;

    if(!inited) PosStore_Init();
    if(!usable) return;

    mech = Motion_PosMechRad();

    /* 判"停住"用**净位移**, 不用瞬时速度:
     * 2026-10-23 实测: 位置环保持时, 20 kHz 下的速度估计在零附近抖,
     * 用速度门限会一直判"还在动"; 而轴其实原地站着 (净位移 < 0.1°)。
     * 3 s 窗口内相对 pos_mark 的净位移小于 POS_STILL_WIN_RAD 就算停住。 */
    if(Motion_Busy())
    {
        t_mark   = now_us;
        pos_mark = mech;
        pending  = 1u;
        return;
    }

    if(mech > pos_mark + POS_STILL_WIN_RAD || mech < pos_mark - POS_STILL_WIN_RAD)
    {
        t_mark   = now_us;
        pos_mark = mech;
        pending  = 1u;
        return;
    }
    if(!pending) return;
    if(!urgent && (uint32_t)(now_us - t_mark) < POS_SAVE_STILL_US) return;

    if(!changed()) { pending = 0u; urgent = 0u; return; }

    if(do_save()) { pending = 0u; urgent = 0u; }
}

void PosStore_PrintInfo(void)
{
    if(!inited) PosStore_Init();

    printf("POS: %s seq=%lu ok=%lu fail=%lu home=%u busy=%u\r\n",
           usable ? "ON" : "OFF", (unsigned long)last_seq,
           (unsigned long)n_ok, (unsigned long)n_fail,
           (unsigned)home_loaded, (unsigned)Motion_Busy());
    printf("     saved mech=%s zero=%s pending=%u%s\r\n",
           FmtF(saved_mech, 3), FmtF(Enc_ZeroRad(), 3),
           (unsigned)pending, urgent ? " (urgent)" : "");
}