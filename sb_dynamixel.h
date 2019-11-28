#ifndef SB_DYNAMIXEL_H
#define SB_DYNAMIXEL_H

#include <ArduinoJson.h>

#define DYNAMIXEL_BAUD 1000000 // default value is 57600, but want to increase this
#define DYNAMIXEL_SERIAL Serial1
#define DYNAMIXEL_IO_SELECT 23
#define IN_BUF_SIZE 10000





typedef struct {
  int curr_pos;
  short goalCurrent;
  int goalPosition;
  int id;
}  Dynamixel_Type;


union ushortToChar {
  unsigned char as_uchar[2];
  unsigned short as_ushort;
};

union shortToChar {
  unsigned char as_uchar[2];
  short as_short;
};

union intToChar {
  unsigned char as_uchar[4];
  unsigned int as_int;
};

union uintToChar {
  unsigned char as_uchar[4];
  unsigned int as_uint;
};

struct INT_UPDATED {
  int curr = 0;
  int last = 0;
  bool changed = 0;
};

struct BOOL_UPDATED {
  bool curr = 0;
  bool last = 0;
  bool changed = 0;
};

struct DOUBLE_UPDATED {
  double curr = 0;
  double last = 0;
  bool changed = 0;
};


class SB_Dynamixel 
{
  public:
    

    SB_Dynamixel(unsigned char device_id, int curr_pos, short goalCurrent, int goalPosition);
    void init();
    unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
    void generic_write(unsigned char address[2], unsigned char payload[], unsigned short n_payload);
    void led_msg(unsigned int led_cmd);
    void operating_mode(unsigned int mode);
    void position_goal();
    void current_goal();
    void torque_en(unsigned char desired_status);
    int read_current();
    int read_position();
    void set_goal_position(int goal_pos);
    void set_goal_current(short goal_current);
    int get_goal_position();
    short get_goal_current();
    double sind(double x);
    double cosd(double x);
    double posIntToDeg(int posInInt, int zeroOffset);
    int posDegToInt(double posInDeg, int zeroOffset);



  private:
    unsigned char m_device_id;
    int m_curr_pos;
    short m_goal_current;
    int m_goal_position;

    unsigned char input_buffer[IN_BUF_SIZE] = {};
    unsigned int buffer_idx;

    const int end_delay_us; // this is a tiny delay after the Serial flush in order to make sure all data is sent before pulling the half-duplex RX/TX select line low
    const int wait_micros; // this is a delay after a read function in order to allow the Dynamixel to respond
};


#endif







