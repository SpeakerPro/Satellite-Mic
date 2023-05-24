#pragma once

void debug_init(const char* file);
int debug_printf(const char* format, ...);
int debug_log(const char* format, ...);
void debug_uninit(void);
