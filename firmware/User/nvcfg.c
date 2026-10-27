#include "nvcfg.h"
#include "board.h"
#include "debug.h"
#include "fmt.h"
#include "ch32l103_flash.h"

/* =============================================================================
 * 掉电保存的配置 —— 说明见 nvcfg.h
 *
 * 擦写粒度: ch32l103_flash.c 里 FLASH_ErasePage_Fast 官方注释
 *           "Erases a specified FLASH page (1page = 256Byte)";
 *           FLASH_BufLoad + FLASH_ProgramPage_Fast 一次写满整页。
 * ========================================================================== */

#define NVCFG_BASE        0x0800FC00u
#define NVCFG_REC_WORDS   16u
#define NVCFG_RECS        4u
#define NVCFG_MAGIC       0x564E3230u          /* 'VN20' */

static uint32_t shadow[64];                    /* 整页 256B 影子 */
static int      inited;
static int      usable;
static int      cur_slot;
static uint32_t next_seq;
static uint32_t n_ok, n_fail;

static uint32_t flash_off(uint32_t a) { return a & 0x00FFFFFFu; }

/* 程序镜像在 Flash 里的末尾地址 —— 顶到记录区就停用, 免得把自己擦掉。 */
static uint32_t code_flash_end(void)
{
    extern char _data_lma[];
    extern char _data_vma[];
    extern char _edata[];

    return (uint32_t)_data_lma + (uint32_t)(_edata - _data_vma);
}

static uint32_t rd32(uint32_t a) { return *(volatile uint32_t *)a; }

static void page_program(void)
{
    uint32_t i;

    FLASH_Unlock_Fast();
    FLASH_BufReset();
    for(i = 0; i < 64u; i++) FLASH_BufLoad(NVCFG_BASE + 4u * i, shadow[i]);
    FLASH_ProgramPage_Fast(NVCFG_BASE);
    FLASH_Lock_Fast();
}

static void page_erase(void)
{
    uint32_t i;

    FLASH_Unlock_Fast();
    FLASH_ErasePage_Fast(NVCFG_BASE);
    FLASH_Lock_Fast();
    for(i = 0; i < 64u; i++) shadow[i] = 0xFFFFFFFFu;
}

static void page_read(void)
{
    uint32_t i;
    for(i = 0; i < 64u; i++) shadow[i] = rd32(NVCFG_BASE + 4u * i);
}

static uint32_t *rec(int slot) { return &shadow[(uint32_t)slot * NVCFG_REC_WORDS]; }

static int rec_valid(int slot)
{
    return (rec(slot)[0] == NVCFG_MAGIC) ? 1 : 0;
}

static int find_best(void)
{
    int i, best = -1;
    uint32_t best_seq = 0u;

    for(i = 0; i < (int)NVCFG_RECS; i++)
    {
        if(!rec_valid(i)) continue;
        if(best < 0 || rec(i)[1] > best_seq)
        {
            best = i;
            best_seq = rec(i)[1];
        }
    }
    return best;
}

/* 页里有没有非 0xFF 的字 (老格式或脏数据) —— 有就说明第一次写之前必须擦 */
static int page_dirty(void)
{
    uint32_t i;
    for(i = 0; i < 64u; i++)
        if(shadow[i] != 0xFFFFFFFFu) return 1;
    return 0;
}

void NvCfg_Init(void)
{
    int best;

    inited = 1;
    cur_slot = 0;
    next_seq = 1u;
    n_ok = n_fail = 0u;

    if(flash_off(code_flash_end()) >= flash_off(NVCFG_BASE))
    {
        usable = 0;
        printf("NVCFG: DISABLED - code end 0x%06lX reached record area 0x%06lX\r\n",
               (unsigned long)flash_off(code_flash_end()),
               (unsigned long)flash_off(NVCFG_BASE));
        return;
    }
    usable = 1;

    page_read();
    best = find_best();
    if(best >= 0)
    {
        next_seq = rec(best)[1] + 1u;
        cur_slot = best + 1;
        if(cur_slot >= (int)NVCFG_RECS) cur_slot = -1;   /* 满了, 下次先擦页 */
    }
    else if(page_dirty())
    {
        cur_slot = -1;                                   /* 老格式/脏页: 先擦再写 */
    }
}

int NvCfg_Enabled(void) { return usable; }

int NvCfg_Load(NvCfg_t *c)
{
    int best;
    uint32_t *r;
    uint32_t t;

    if(!inited) NvCfg_Init();
    if(!usable || !c) return 0;
    best = find_best();
    if(best < 0) return 0;

    r = rec(best);
    c->node      = (int)r[2];
    c->pp        = (int)r[3];
    c->ilim_ca   = (int)r[4];
    c->vbus_cv   = (int)r[5];
    c->spd_kp_e5 = (int)r[6];
    c->spd_ki_e2 = (int)r[7];
    c->pos_kp_e2 = (int)r[8];
    c->cur_kp_e4 = (int)r[9];
    c->cur_ki_e2 = (int)r[10];
    c->vel_lim_e1= (int)r[11];
    c->accel_e1  = (int)r[12];
    t            = r[13];
    c->align_mv  = (int)(t & 0x3FFFu);                 /* 低 14 位: 对齐电压 mV */
    c->oc_trip_ca= (int)((t >> 14) & 0x3FFu);          /* 位14..23: 跳闸阈值 0.01A (老记录=0) */
    c->vsrc      = (int)((t >> 24) & 0xFFu);
    c->pll_bw_e1 = (int)(r[14] & 0xFFFFu);
    /* 高 16 位存 oc_trip_ms+1 (0 = 老记录没这一项, 用默认; 不能直接存 0, 因为 0ms 是合法值) */
    c->oc_trip_ms = ((r[14] >> 16) != 0u) ? (int)((r[14] >> 16) - 1u) : -1;
    t            = r[15];
    c->r_mohm    = (int)(t & 0xFFFFu);
    c->l_uh      = (int)((t >> 16) & 0xFFFFu);
    return 1;
}

int NvCfg_Save(const NvCfg_t *c)
{
    NvCfg_t o;
    uint32_t seq, *r;
    int slot, have_old;

    if(!inited) NvCfg_Init();
    if(!usable || !c) return 0;

    have_old = NvCfg_Load(&o);
    if(!have_old) { o.node=o.pp=o.ilim_ca=o.vbus_cv=0;
                    o.spd_kp_e5=o.spd_ki_e2=o.pos_kp_e2=0;
                    o.cur_kp_e4=o.cur_ki_e2=0;
                    o.vel_lim_e1=o.accel_e1=o.align_mv=0;
                    o.pll_bw_e1=o.r_mohm=o.l_uh=o.vsrc=o.oc_trip_ca=0;
                    o.oc_trip_ms=-1; }

    if(cur_slot < 0)
    {
        page_erase();
        cur_slot = 0;
    }

    slot = cur_slot;
    seq  = next_seq++;
    r    = rec(slot);

    r[0]  = NVCFG_MAGIC;
    r[1]  = seq;
    r[2]  = (uint32_t)(c->node      ? c->node      : o.node);
    r[3]  = (uint32_t)(c->pp        ? c->pp        : o.pp);
    r[4]  = (uint32_t)(c->ilim_ca   ? c->ilim_ca   : o.ilim_ca);
    r[5]  = (uint32_t)(c->vbus_cv   ? c->vbus_cv   : o.vbus_cv);
    r[6]  = (uint32_t)(c->spd_kp_e5 ? c->spd_kp_e5 : o.spd_kp_e5);
    r[7]  = (uint32_t)(c->spd_ki_e2 ? c->spd_ki_e2 : o.spd_ki_e2);
    r[8]  = (uint32_t)(c->pos_kp_e2 ? c->pos_kp_e2 : o.pos_kp_e2);
    r[9]  = (uint32_t)(c->cur_kp_e4 ? c->cur_kp_e4 : o.cur_kp_e4);
    r[10] = (uint32_t)(c->cur_ki_e2 ? c->cur_ki_e2 : o.cur_ki_e2);
    r[11] = (uint32_t)(c->vel_lim_e1? c->vel_lim_e1: o.vel_lim_e1);
    r[12] = (uint32_t)(c->accel_e1  ? c->accel_e1  : o.accel_e1);
    r[13] = ((uint32_t)(c->align_mv ? c->align_mv : o.align_mv) & 0x3FFFu)
          | (((uint32_t)(c->oc_trip_ca ? c->oc_trip_ca : o.oc_trip_ca) & 0x3FFu) << 14)
          | (((uint32_t)(c->vsrc      ? c->vsrc     : o.vsrc) & 0xFFu) << 24);
    r[14] = ((uint32_t)(c->pll_bw_e1 ? c->pll_bw_e1 : o.pll_bw_e1) & 0xFFFFu)
          | ((uint32_t)((c->oc_trip_ms >= 0 ? c->oc_trip_ms : (o.oc_trip_ms >= 0 ? o.oc_trip_ms : (int)MOTOR_OC_TRIP_MS_DEF)) + 1) << 16);
    r[15] = ((uint32_t)(c->r_mohm ? c->r_mohm : o.r_mohm) & 0xFFFFu)
          | (((uint32_t)(c->l_uh   ? c->l_uh   : o.l_uh) & 0xFFFFu) << 16);

    page_program();

    /* 回读核对 —— 写失败要能被看见, 不许"以为写进去了" */
    page_read();
    if(rec_valid(slot) && rec(slot)[1] == seq && rec(slot)[2] == r[2] && rec(slot)[3] == r[3])
    {
        n_ok++;
        cur_slot = slot + 1;
        if(cur_slot >= (int)NVCFG_RECS) cur_slot = -1;
        return 1;
    }

    n_fail++;
    return 0;
}

int NvCfg_Erase(void)
{
    if(!inited) NvCfg_Init();
    if(!usable) return 0;
    page_erase();
    cur_slot = 0;
    next_seq = 1u;
    return 1;
}

uint32_t NvCfg_WriteOk(void)   { return n_ok; }
uint32_t NvCfg_WriteFail(void) { return n_fail; }
uint32_t NvCfg_Seq(void)       { return next_seq; }

void NvCfg_PrintInfo(void)
{
    NvCfg_t c;

    if(!inited) NvCfg_Init();
    printf("NVCFG: %s base=0x%06lX slot=%d seq=%lu ok=%lu fail=%lu\r\n",
           usable ? "ON" : "OFF", (unsigned long)flash_off(NVCFG_BASE),
           cur_slot, (unsigned long)next_seq,
           (unsigned long)n_ok, (unsigned long)n_fail);
    if(NvCfg_Load(&c))
    {
        /* FmtF 的环形缓冲只有 10 个, 一条 printf 里最多放 3~4 个, 否则前面的会被覆盖 */
        printf("NVCFG: node=%d pp=%d ilim=%s A vbus=%s V\r\n",
               c.node, c.pp, FmtF((float)c.ilim_ca * 0.01f, 2), FmtF((float)c.vbus_cv * 0.01f, 2));
        printf("NVCFG: spd %s/%s pos %s\r\n",
               FmtF((float)c.spd_kp_e5 * 0.00001f, 5), FmtF((float)c.spd_ki_e2 * 0.01f, 2),
               FmtF((float)c.pos_kp_e2 * 0.01f, 2));
        printf("NVCFG: cur %s/%s\r\n",
               FmtF((float)c.cur_kp_e4 * 0.0001f, 4), FmtF((float)c.cur_ki_e2 * 0.01f, 1));
        printf("NVCFG: R=%s L=%smH vlim=%s acc=%s\r\n",
               FmtF((float)c.r_mohm * 0.001f, 2), FmtF((float)c.l_uh * 0.001f, 3),
               FmtF((float)c.vel_lim_e1 * 0.1f, 1), FmtF((float)c.accel_e1 * 0.1f, 1));
        printf("NVCFG: align=%sV pll=%sHz vs=%d\r\n",
               FmtF((float)c.align_mv * 0.001f, 2), FmtF((float)c.pll_bw_e1 * 0.1f, 1), c.vsrc);
        /* FmtF 环形缓冲只有 10 个, 单独一行放第 4 个值 */
        printf("NVCFG: oc_trip=%s A\r\n", FmtF((float)c.oc_trip_ca * 0.01f, 2));
        /* 单独一行: FmtF 环形缓冲只有 10 个, 一行最多放 3~4 个值 */
        printf("NVCFG: oc_trip_ms=%d ms\r\n", c.oc_trip_ms >= 0 ? c.oc_trip_ms : (int)MOTOR_OC_TRIP_MS_DEF);
    }
    else
        printf("NVCFG: (no record) - using firmware defaults\r\n");
}
