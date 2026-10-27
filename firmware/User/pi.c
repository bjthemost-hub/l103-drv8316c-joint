#include "pi.h"

void PI_Init(PI_t *p, float kp, float ki, float ts, float lo, float hi)
{
    p->kp      = kp;
    p->ki      = ki;
    p->ts      = ts;
    p->integ   = 0.0f;
    p->out_min = lo;
    p->out_max = hi;
}

void PI_Reset(PI_t *p)          { p->integ = 0.0f; }
void PI_SetGains(PI_t *p, float kp, float ki) { p->kp = kp; p->ki = ki; }

float PI_RunLim(PI_t *p, float err, float lo, float hi)
{
    float out = p->kp * err + p->integ;

    if (out > hi)      { out = hi; p->integ = hi - p->kp * err; }
    else if (out < lo) { out = lo; p->integ = lo - p->kp * err; }

    p->integ += p->ki * p->ts * err;
    return out;
}

float PI_Run(PI_t *p, float err)
{
    return PI_RunLim(p, err, p->out_min, p->out_max);
}