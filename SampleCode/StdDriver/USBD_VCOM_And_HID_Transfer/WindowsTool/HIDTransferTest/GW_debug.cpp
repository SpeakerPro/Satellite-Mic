#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "stdafx.h"

#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

FILE* fp = NULL;
void debug_init(const char* file)
{
    if(file != NULL)
        fp = fopen(file, "wb+");
}

int debug_printf(const char* format, ...)
{
    char    buffer[256] = {};
    int     len = 0;
    va_list ap;

    va_start(ap, format);
    vsnprintf(buffer, sizeof(buffer), format, ap);
    va_end(ap);

    if (fp != NULL)
    {
        len = fprintf(fp, "%s", (const char*)buffer);
        fflush(fp);
    }

    len = fprintf(stdout, "%s", (const char*)buffer);
    fflush(stdout);

    return len;
}

int debug_log(const char* format, ...)
{
    if (fp == NULL)
        return 0;

    char    buffer[256] = {};
    int     len;
    va_list ap;

    va_start(ap, format);
    vsnprintf(buffer, sizeof(buffer), format, ap);
    va_end(ap);

    len = fprintf(fp, "%s", (const char*)buffer);
    fflush(fp);

    return len;
}

void debug_uninit(void)
{
    if (fp != NULL)
        fclose(fp);
}
