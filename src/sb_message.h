#ifndef SB_MESSAGE_H
#define SB_MESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

uint8_t motorAddrSpace = 0x10;

typedef enum SBTypes {
    TYPE_WRITE = 0xAA,
    TYPE_READ = 0xBB,
    TYPE_STREAM = 0xCC,
    TYPE_RESPONSE = 0xDD,
    TYPE_ERROR = 0xEE
} SBTypes;


#define REG_REL_EN  0x0000
#define REG_REL_POS  0x0001
#define REG_REL_SPEED  0x0002
#define REG_REL_KP  0x0003
#define REG_REL_KI  0x0004
#define REG_REL_KD  0x0005
#define REG_REL_MOVE  0x0006
#define REG_REL_STOP  0x0007
#define REG_REL_GOAL  0x0008
#define REG_REL_INIT  0x0009
#define REG_REL_MODE 0x000A


typedef enum SBRegisters {
    REG_INIT = 0x0000,
    REG_FIRM_VERSION = 0x0001,
    REG_HARD_VERSION = 0x0002,
    REG_TIME = 0x0003,

    REG_M1_START = 0x0010,
    REG_M2_START = 0x0020,
    REG_M3_START = 0x0030,
    REG_M4_START = 0x0040,
    REG_M5_START = 0x0050,
    REG_M6_START = 0x0060,
    REG_M7_START = 0x0070,
    REG_M8_START = 0x0080,
    REG_M9_START = 0x0090,
    REG_MA_START = 0x00A0,
    REG_MB_START = 0x00B0,
    REG_MC_START = 0x00C0,



    REG_ALL_POS = 0x00D1,
    REG_ALL_SPEED = 0x00D2,

//    REG_MOVE = 0x00D5,

    REG_RETURN_VALS = 0x00E0,
    REG_NA = 0xFF00
} SBRegisters;

typedef enum SBError {
    ERR_DEBUG = 0x0000,
    ERR_PC_CRC = 0x0001,
    ERR_DYN_CRC = 0x0002,
    ERR_UNSUPPORTED_REG = 0x0003,
    ERR_BUSY = 0x0003,
} SBError;

#pragma pack(push, 1)
typedef struct {
    uint32_t time;
    float data;
} TimestampedFloat;

typedef struct {
    uint32_t time;
    float data1;
    float data2;
    float data3;
    float data4;
} Timestamped4Floats;

typedef struct {
    uint32_t time;
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
    float data7;
    float data8;
    float data9;
} Timestamped9Floats;

typedef struct {
    uint32_t time;
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
    float data7;
    float data8;
    float data9;
    float data10;
    float data11;
    float data12;
} Timestamped12Floats;

typedef struct {
    uint32_t time;
    double data1;
    double data2;
    double data3;
    double data4;
} Timestamped4Doubles;

typedef struct {
    uint32_t time;
    double data1;
    double data2;
    double data3;
    double data4;
    double data5;
    double data6;
} Timestamped6Doubles;

typedef struct {
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
    float data7;
    float data8;
    float data9;
    float data10;
    float data11;
    float data12;
    short current1;
    short current2;
    short current3;
    short current4;
} FullState12Floats4Shorts;

#pragma pack(pop)


extern void sb_stuff(uint8_t *msg);
extern void sb_unstuff(uint8_t *msg);

inline uint8_t sb_get_length(uint8_t *msg) {
    return msg[1];
}

inline void sb_set_length(uint8_t *msg, uint8_t length) {
    msg[1] = length;
}

inline void sb_set_data_length(uint8_t *msg, uint8_t length) {
    sb_set_length(msg, (uint16_t) length + 6);
}

inline SBTypes sb_get_type(uint8_t *msg) {
    return (SBTypes) msg[3];
}

inline void sb_set_type(uint8_t *msg, SBTypes type) {
    msg[3] = (uint8_t) type;
}

inline SBRegisters sb_get_register(uint8_t *msg) {
    uint16_t tmp = msg[4] + (msg[5] << 8);
    return (SBRegisters) tmp;
}

inline void sb_set_register(uint8_t *msg, SBRegisters reg) {
    msg[4] = reg; // Assume little endian / correct truncation behavior
    msg[5] = (reg >> 8);
}

inline TimestampedFloat *sb_get_data_TimestampedFloat(uint8_t *msg) {
    return (TimestampedFloat *) &msg[6];
}

inline void sb_set_data_TimestampedFloat(uint8_t *msg, TimestampedFloat data) {
    memcpy(&msg[6], &data, sizeof(TimestampedFloat));
    sb_set_data_length(msg, sizeof(TimestampedFloat));
}

inline Timestamped4Floats *sb_get_data_Timestamped4Floats(uint8_t *msg) {
    return (Timestamped4Floats *) &msg[6];
}

inline void sb_set_data_Timestamped4Floats(uint8_t *msg, Timestamped4Floats data) {
    memcpy(&msg[6], &data, sizeof(Timestamped4Floats));
    sb_set_data_length(msg, sizeof(Timestamped4Floats));
}

inline void sb_set_data_Timestamped4Doubles(uint8_t *msg, Timestamped4Doubles data) {
    memcpy(&msg[6], &data, sizeof(Timestamped4Doubles));
    sb_set_data_length(msg, sizeof(Timestamped4Doubles));
}

inline void sb_set_data_Timestamped9Floats(uint8_t *msg, Timestamped9Floats data) {
    memcpy(&msg[6], &data, sizeof(Timestamped9Floats));
    sb_set_data_length(msg, sizeof(Timestamped9Floats));
}

inline void sb_set_data_Timestamped12Floats(uint8_t *msg, Timestamped12Floats data) {
    memcpy(&msg[6], &data, sizeof(Timestamped12Floats));
    sb_set_data_length(msg, sizeof(Timestamped12Floats));
}

inline void sb_set_data_FullState12Floats4Shorts(uint8_t *msg, FullState12Floats4Shorts data) {
    memcpy(&msg[6], &data, sizeof(FullState12Floats4Shorts));
    sb_set_data_length(msg, sizeof(FullState12Floats4Shorts));
}

inline char *sb_get_data_str(uint8_t *msg) {
    return reinterpret_cast<char *>(msg + 6);
}

inline void sb_set_data_str(uint8_t *msg, char *data) {
    strcpy((char *) &msg[6], data);
    sb_set_data_length(msg, strlen(data) + 1);
}

inline void sb_set_data(uint8_t *msg, void *data, size_t size) {
    memcpy(&msg[6], data, size);
    sb_set_data_length(msg, size);
}

#ifdef __cplusplus
}
#endif

#endif //SB_MESSAGE_H
