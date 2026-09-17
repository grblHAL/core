/*
  strutils.c - a collection of useful string utilities

  Part of grblHAL

  Copyright (c) 2019-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "grbl.h"
#include "strutils.h"

#if defined(_WIN32) || defined(__MSP432P401R__) || defined(PART_TM4C123GH6PM)

FLASHMEM size_t strlcpy (char *dst, const char *src, size_t len)
{
    const char *s = src;
    size_t dlen = len;

    if(dlen) while(--dlen) {
        if(!(*dst++ = *s++))
            break;
    }

    if(dlen == 0) {
        if(len)
            *dst = '\0';
        while(*s++);
    }

    return s - src - 1;
}

#endif // _WIN32 || __MSP432P401R__ || PART_TM4C123GH6PM

/*! \brief Case insensitive search for first occurrence of a string inside another.
\param s1 pointer to string to search.
\param s2 pointer to string to search for.
\returns pointer to found string or NULL if not found.
*/
FLASHMEM char *stristr (const char *s1, const char *s2)
{
    const char *s = s1, *p = s2, *r = NULL;

    if (!s2 || strlen(s2) == 0)
        return (char *)s1;

    while(*s && *p) {

        if(CAPS(*p) == CAPS(*s)) {
            if(!r)
                r = s;
            p++;
        } else {
            p = s2;
            if(r)
                s = r + 1;
            if(CAPS(*p) == CAPS(*s)) {
                r = s;
                p++;
            } else
                r = NULL;
        }
        s++;
    }

    return *p ? NULL : (char *)r;
}

/*! \brief Case insensitive search for first occurrence of a string inside another.
\param s1 pointer to string to search.
\param s2 pointer to string to search for.
\param len max. number of characters to compare.
\returns pointer to found string or NULL if not found.
*/
FLASHMEM char *strnistr (const char *s1, const char *s2, size_t len)
{
    size_t slen = len;
    const char *s = s1, *p = s2, *r = NULL;

    if (!s2 || strlen(s2) == 0 || len == 0)
        return (char *)s1;

    while(slen && *s && *p) {

        if(CAPS(*p) == CAPS(*s)) {
            if(!r)
                r = s;
            p++;
        } else {
            p = s2;
            slen = len;
            if(r)
                s = r + 1;
            if(CAPS(*p) == CAPS(*s)) {
                r = s;
                p++;
            } else
                r = NULL;
        }
        s++;
        if(r)
            slen--;
    }

    return slen == 0 || *p == '\0' ? (char *)r : NULL;
}

// NOTE: Ensure buf is large enough to hold concatenated strings!
//       Do NOT use for several int/float conversions as these share the same underlying buffer!
FLASHMEM char *strappend (char *buf, int argc, ...)
{
    char c, *s = buf, *arg;

    va_list list;
    va_start(list, argc);

    while(argc--) {
        arg = va_arg(list, char *);
        do {
            c = *s++ = *arg++;
        } while(c);
        s--;
    }

    va_end(list);

    return buf;
}

FLASHMEM uint32_t strnumentries (const char *s, const char delimiter)
{
    char *p = (char *)s;
    uint32_t entries = (s && *s) ? 1 : 0;

    while(entries && (p = strchr(p, delimiter))) {
        p++;
        entries++;
    }

    return entries;
}

FLASHMEM char *strgetentry (char *res, const char *s, uint32_t entry, const char delimiter)
{
    char *e, *p = (char *)s;

    *res = '\0';
    e = strchr(p, delimiter);

    if(entry == 0 && e == NULL)
        strcpy(res, s);
    else do {
        if(entry) {
            p = e;
            e = strchr(++p, delimiter);
            entry--;
        } else {
            if(e) {
                strncpy(res, p,  e - p);
                res[e - p] = '\0';
            } else
                strcpy(res, p);
            break;
        }
    } while(true);

    return res;
}

FLASHMEM int32_t strlookup (const char *s1, const char *s2, const char delimiter)
{
    bool found = false;
    char *e, *p = (char *)s2;
    uint32_t idx = strnumentries(s2, delimiter), len = strlen(s1);
    int32_t entry = 0;

    while(idx--) {

        if((e = strchr(p, delimiter)))
            found = (e - p) == len && !strncmp(p, s1, e - p);
        else
            found = strlen(p) == len && !strcmp(p, s1);

        if(found || e == NULL)
            break;
        else {
            p = e + 1;
            entry++;
        }
    }

    return found ? entry : -1;
}

FLASHMEM bool strtotime (char *s, struct tm *time)
{
    char c, *s1 = s;
    uint_fast16_t idx = 0;

    if(s == NULL || time == NULL)
        return false;

    while((c = *s1)) {
        if(c == ':')
            *s1 = ' ';
        s1++;
    }

    if(strchr(s, ',')) { // "text format"

        s1 = strtok(s + 5, " ");
        while(s1) {

            switch(idx) {

                case 0:
                    time->tm_mday = atoi(s1);
                    break;

                case 1:
                    time->tm_mon = strlookup(s1, "Jan,Feb,Mar,Apr,May,Jun,Jul,Aug,Sep,Oct,Nov,Dec", ',');
                    break;

                case 2:
                    time->tm_year = atoi(s1) - 1900;
                    break;

                case 3:
                    time->tm_hour = atoi(s1);
                    break;

                case 4:
                    time->tm_min = atoi(s1);
                    break;

                case 5:
                    time->tm_sec = atoi(s1);
                    break;
            }
            idx++;
            s1 = strtok(NULL, " ");
        }
    } else { // ISO8601 format

        s1 = s;

        while((c = *s1)) {
            if(c == '-' || c == 'T')
                *s1 = ' ';
            else if(c == 'Z')
                *s1 = '\0';
            s1++;
        }

        s1 = strtok(s, " ");
        while(s1) {

            switch(idx) {

                case 0:
                    time->tm_year = atoi(s1) - 1900;
                    break;

                case 1:
                    time->tm_mon = atoi(s1) - 1;
                    break;

                case 2:
                    time->tm_mday = atoi(s1);
                    break;

                case 3:
                    time->tm_hour = atoi(s1);
                    break;

                case 4:
                    time->tm_min = atoi(s1);
                    break;

                case 5:
                    time->tm_sec = atoi(s1);
                    break;
            }
            idx++;
            s1 = strtok(NULL, " ");
        }
    }

    return idx >= 5;
}

FLASHMEM char *strtoisodt (struct tm *dt)
{
    static char buf[21];

    if(dt == NULL || snprintf(buf, sizeof(buf), "%04i-%02i-%02iT%02i:%02i:%02i", dt->tm_year < 1000 ? dt->tm_year + 1900 : dt->tm_year, dt->tm_mon + 1, dt->tm_mday, dt->tm_hour, dt->tm_min, dt->tm_sec) > sizeof(buf))
        *buf = '\0';

    return buf;
}

FLASHMEM char *strtointernetdt (struct tm *dt)
{
    PROGMEM static const char *month_table[12] = {
        "Jan",
        "Feb",
        "Mar",
        "Apr",
        "May",
        "Jun",
        "Jul",
        "Aug",
        "Sep",
        "Oct",
        "Nov",
        "Dec"
    };

    PROGMEM static const char *day_table[7] = {
        "Sun",
        "Mon",
        "Tue",
        "Wed",
        "Thu",
        "Fri",
        "Sat"
    };

    static char buf[31];

#ifdef __MSP432E401Y__
    return "Thu, 01 Jan 1970 00:00:00 GMT"; // snprintf below crashes the MCU!
#else

    if(dt == NULL || snprintf(buf, sizeof(buf), "%s, %02d %s %04d %02d:%02d:%02d GMT", day_table[dt->tm_wday], dt->tm_mday, month_table[dt->tm_mon], dt->tm_year < 1000 ? dt->tm_year + 1900 : dt->tm_year, dt->tm_hour, dt->tm_min, dt->tm_sec) > sizeof(buf))
        return "Thu, 01 Jan 1970 00:00:00 GMT";

    return buf;

#endif
}

FLASHMEM char *btoa (uint64_t bytes)
{
    static char buf[16];

    uint_fast8_t n = 0;
    uint64_t size = bytes;

    while(size > 1024) {
        size >>= 10;
        n++;
    }

    strcpy(buf, ftoa((float)bytes / (float)(1ULL << 10 * n), n ? 2 : 0));

    if(n == 0) // remove trailing decimal point...
        buf[strlen(buf) - 1] = '\0';

    strcat(buf, n == 0 ? " B" : n == 1 ? " KB" : n == 2 ? " MB" : " GB");

    return buf;
}
