#include "fmt.h"

#define FMT_RING    10

static char    fmt_ring[FMT_RING][20];
static uint8_t fmt_idx;

static const char *fmt_impl(float v, uint8_t dec, uint8_t force_sign)
{
    char     *o = fmt_ring[fmt_idx];
    char     *p = o;
    char      tmp[12];
    uint32_t  scale = 1u;
    uint32_t  a, ip, fp;
    int32_t   s;
    uint8_t   i, n;

    fmt_idx = (uint8_t)((fmt_idx + 1u) % FMT_RING);

    if(dec > 6u) dec = 6u;
    for(i = 0; i < dec; i++) scale *= 10u;

    s = (int32_t)(v * (float)scale + (v < 0.0f ? -0.5f : 0.5f));
    if(s < 0) { *p++ = '-'; a = (uint32_t)(-s); }
    else      { if(force_sign) *p++ = '+'; a = (uint32_t)s; }

    ip = a / scale;
    fp = a % scale;

    n = 0;
    if(ip == 0u) tmp[n++] = '0';
    while(ip > 0u) { tmp[n++] = (char)('0' + (ip % 10u)); ip /= 10u; }
    while(n > 0u) *p++ = tmp[--n];

    if(dec > 0u)
    {
        *p++ = '.';
        n = dec;
        while(n > 0u) { n--; tmp[n] = (char)('0' + (fp % 10u)); fp /= 10u; }
        for(i = 0; i < dec; i++) *p++ = tmp[i];
    }
    *p = '\0';
    return o;
}

const char *FmtF(float v, uint8_t dec)  { return fmt_impl(v, dec, 0); }
const char *FmtFS(float v, uint8_t dec) { return fmt_impl(v, dec, 1); }