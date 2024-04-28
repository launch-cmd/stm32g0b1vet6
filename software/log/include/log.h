#ifndef _LOG_H__
#define _LOG_H__

#include <stdio.h>

#define LOG_ERROR(f_, ...) printf(("\x1b[31m"            \
                                   "ERROR: " f_ "\r\n"), \
                                  ##__VA_ARGS__)
#define LOG_WARN(f_, ...) printf(("\x1b[33m"           \
                                  "WARN: " f_ "\r\n"), \
                                 ##__VA_ARGS__)
#define LOG_INFO(f_, ...) printf(("\x1b[37m"           \
                                  "INFO: " f_ "\r\n"), \
                                 ##__VA_ARGS__)
#define LOG_DEBUG(f_, ...) printf(("\x1b[32m"            \
                                   "DEBUG: " f_ "\r\n"), \
                                  ##__VA_ARGS__)

void uartInit();
void uartEnable();
void uartWriteByte(char b);
void uartWriteLine(const char *bytes);
#endif