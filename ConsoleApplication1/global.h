#pragma once
#define _POSIX_C_SOURCE 200112L


#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>


#define MAX_POS 8
#define MAX_PORT 2

extern uint16_t myPos;

typedef struct {
    uint8_t subnet[MAX_PORT];   // 0 if unused
} NodeCfg;

extern NodeCfg topology[MAX_POS]; // topology from config



