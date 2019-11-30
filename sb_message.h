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
#define REG_REL_CURRENT 0x000A




typedef enum SBRegisters {
  REG_INIT = 0x0000,
  REG_FIRM_VERSION = 0x0001,
  REG_HARD_VERSION = 0x0002,
  REG_TIME = 0x0003,
  
  REG_M1_EN = 0x0010,
  REG_M1_POS = 0x0011,
  REG_M1_SPEED = 0x0012,
  REG_M1_KP = 0x0013,
  REG_M1_KI = 0x0014,
  REG_M1_KD = 0x0015,
  REG_M1_MOVE = 0x0016,
  REG_M1_STOP = 0x0017,
  REG_M1_GOAL  = 0x0018,
  REG_M1_INIT = 0x0019,

  REG_M2_EN = 0x0020,
  REG_M2_POS = 0x0021,
  REG_M2_SPEED = 0x0022,
  REG_M2_KP = 0x0023,
  REG_M2_KI = 0x0024,
  REG_M2_KD = 0x0025,
  REG_M2_MOVE = 0x0026,
  REG_M2_STOP = 0x0027,
  REG_M2_GOAL  = 0x0028,
  REG_M2_INIT = 0x0029,

  REG_M3_EN = 0x0030,
  REG_M3_POS = 0x0031,
  REG_M3_SPEED = 0x0032,
  REG_M3_KP = 0x0033,
  REG_M3_KI = 0x0034,
  REG_M3_KD = 0x0035,
  REG_M3_MOVE = 0x0036,
  REG_M3_STOP = 0x0037,
  REG_M3_GOAL  = 0x0038,
  REG_M3_INIT = 0x0039,

  REG_M4_EN = 0x0040,
  REG_M4_POS = 0x0041,
  REG_M4_SPEED = 0x0042,
  REG_M4_KP = 0x0043,
  REG_M4_KI = 0x0044,
  REG_M4_KD = 0x0045,
  REG_M4_MOVE = 0x0046,
  REG_M4_STOP = 0x0047,
  REG_M4_GOAL  = 0x0048,
  REG_M4_INIT = 0x0049,

  REG_M5_EN = 0x0050,
  REG_M5_POS = 0x0051,
  REG_M5_SPEED = 0x0052,
  REG_M5_KP = 0x0053,
  REG_M5_KI = 0x0054,
  REG_M5_KD = 0x0055,
  REG_M5_MOVE = 0x0056,
  REG_M5_STOP = 0x0057,
  REG_M5_GOAL  = 0x0058,
  REG_M5_INIT = 0x0059,

  REG_M6_EN = 0x0060,
  REG_M6_POS = 0x0061,
  REG_M6_SPEED = 0x0062,
  REG_M6_KP = 0x0063,
  REG_M6_KI = 0x0064,
  REG_M6_KD = 0x0065,
  REG_M6_MOVE = 0x0066,
  REG_M6_STOP = 0x0067,
  REG_M6_GOAL  = 0x0068,
  REG_M6_INIT = 0x0069,

  REG_M7_EN = 0x0070,
  REG_M7_POS = 0x0071,
  REG_M7_SPEED = 0x0072,
  REG_M7_KP = 0x0073,
  REG_M7_KI = 0x0074,
  REG_M7_KD = 0x0075,
  REG_M7_MOVE = 0x0076,
  REG_M7_STOP = 0x0077,
  REG_M7_GOAL  = 0x0078,
  REG_M7_INIT = 0x0079,

  REG_M8_EN = 0x0080,
  REG_M8_POS = 0x0081,
  REG_M8_SPEED = 0x0082,
  REG_M8_KP = 0x0083,
  REG_M8_KI = 0x0084,
  REG_M8_KD = 0x0085,
  REG_M8_MOVE = 0x0086,
  REG_M8_STOP = 0x0087,
  REG_M8_GOAL  = 0x0088,
  REG_M8_INIT = 0x0089,

  REG_M9_EN = 0x0090,
  REG_M9_POS = 0x0091,
  REG_M9_SPEED = 0x0092,
  REG_M9_KP = 0x0093,
  REG_M9_KI = 0x0094,
  REG_M9_KD = 0x0095,
  REG_M9_MOVE = 0x0096,
  REG_M9_STOP = 0x0097,
  REG_M9_GOAL  = 0x0098,
  REG_M9_INIT = 0x0099,

  REG_RETURN_VALS = 0x00A0,

  REG_MOVE  = 0x00A5,
  REG_NA  = 0xFF00
} SBRegisters;

typedef enum SBError {
  ERR_DEBUG = 0x0000,
  ERR_PC_CRC = 0x0001,
  ERR_DYN_CRC = 0x0002,
  ERR_UNSUPPORTED_REG = 0x0003,
  ERR_BUSY = 0x0003,
} SBError;

#pragma pack(push,1)
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

#pragma pack(pop)


extern void sb_stuff(uint8_t* msg);
extern void sb_unstuff(uint8_t* msg);

inline uint8_t sb_get_length(uint8_t* msg) {
  return msg[1];
}

inline void sb_set_length(uint8_t* msg, uint8_t length) {
  msg[1] = length;
}

inline void sb_set_data_length(uint8_t* msg, uint8_t length) {
  sb_set_length(msg, (uint16_t) length + 6);
}

inline SBTypes sb_get_type(uint8_t* msg) {
  return (SBTypes) msg[3];
}

inline void sb_set_type(uint8_t* msg, SBTypes type) {
  msg[3] = (uint8_t) type;
}

inline SBRegisters sb_get_register(uint8_t* msg) {
  uint16_t tmp = msg[4]+(msg[5] << 8);
  return (SBRegisters) tmp;
}

inline void sb_set_register(uint8_t* msg, SBRegisters reg) {
  msg[4] = reg; // Assume little endian / correct truncation behavior
  msg[5] = (reg >> 8);
}

inline TimestampedFloat* sb_get_data_TimestampedFloat(uint8_t* msg) {
  return (TimestampedFloat*) &msg[6];
}

inline void sb_set_data_TimestampedFloat(uint8_t* msg, TimestampedFloat data) {
  memcpy(&msg[6], &data, sizeof(TimestampedFloat));
  sb_set_data_length(msg, sizeof(TimestampedFloat));
}

inline Timestamped4Floats* sb_get_data_Timestamped4Floats(uint8_t* msg) {
  return (Timestamped4Floats*) &msg[6];
}

inline void sb_set_data_Timestamped4Floats(uint8_t* msg, Timestamped4Floats data) {
  memcpy(&msg[6], &data, sizeof(Timestamped4Floats));
  sb_set_data_length(msg, sizeof(Timestamped4Floats));
}

inline void sb_set_data_Timestamped4Doubles(uint8_t* msg, Timestamped4Doubles data) {
    memcpy(&msg[6], &data, sizeof(Timestamped4Doubles));
    sb_set_data_length(msg, sizeof(Timestamped4Doubles));
}

inline void sb_set_data_Timestamped9Floats(uint8_t* msg, Timestamped9Floats data) {
    memcpy(&msg[6], &data, sizeof(Timestamped9Floats));
    sb_set_data_length(msg, sizeof(Timestamped9Floats));
}

inline void sb_set_data_Timestamped12Floats(uint8_t* msg, Timestamped12Floats data) {
    memcpy(&msg[6], &data, sizeof(Timestamped12Floats));
    sb_set_data_length(msg, sizeof(Timestamped12Floats));
}

inline char* sb_get_data_str(uint8_t* msg) {
  return reinterpret_cast<char *>(msg + 6);
}

inline void sb_set_data_str(uint8_t* msg, char* data) {
  strcpy((char*) &msg[6], data);
  sb_set_data_length(msg, strlen(data) + 1);
}

inline void sb_set_data(uint8_t* msg, void* data, size_t size) {
  memcpy(&msg[6], data, size);
  sb_set_data_length(msg, size);
}

#ifdef __cplusplus
}
#endif

#endif //SB_MESSAGE_H
