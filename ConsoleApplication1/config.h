#ifndef CONFIG__
#define CONFIG__

typedef uint16_t PosType_t;
#define MAX_POS 100
typedef struct NodeCfg_st
{
    uint8_t subnet[MAX_PORT]; // 0 if unused
} NodeCfg_t;

typedef struct ConfigFile_st {
    PosType_t pos;
    NodeCfg_t topology[MAX_POS];
} ConfigFile_t;

#endif