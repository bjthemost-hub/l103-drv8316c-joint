#ifndef __FMT_H
#define __FMT_H

#include <stdint.h>

/* 定点打印 (替代 newlib 的浮点 printf, 省 ~5.7KB Flash)
 *   FmtF(v, dec)  -> "123.456"
 *   FmtFS(v, dec) -> 正数也带 '+' ("+123.456")
 * 返回静态环形缓冲里的字符串, 一次 printf 里最多同时用 6 个。
 * 为什么不用 %f: mini_fmt 不支持浮点; newlib 的 %f 会把 vfprintf_i 那一串
 * 全拉进来 (约 6KB), 这块板 63KB Flash 吃不消。 */

const char *FmtF(float v, uint8_t dec);
const char *FmtFS(float v, uint8_t dec);

#endif