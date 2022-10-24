#ifndef _ROS_roboclaw_driver_Stats_h
#define _ROS_roboclaw_driver_Stats_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace roboclaw_driver
{

  class Stats : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _m1_enc_val_type;
      _m1_enc_val_type m1_enc_val;
      typedef int32_t _m2_enc_val_type;
      _m2_enc_val_type m2_enc_val;
      typedef int32_t _m1_enc_qpps_type;
      _m1_enc_qpps_type m1_enc_qpps;
      typedef int32_t _m2_enc_qpps_type;
      _m2_enc_qpps_type m2_enc_qpps;

    Stats():
      header(),
      m1_enc_val(0),
      m2_enc_val(0),
      m1_enc_qpps(0),
      m2_enc_qpps(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_m1_enc_val;
      u_m1_enc_val.real = this->m1_enc_val;
      *(outbuffer + offset + 0) = (u_m1_enc_val.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m1_enc_val.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m1_enc_val.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m1_enc_val.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m1_enc_val);
      union {
        int32_t real;
        uint32_t base;
      } u_m2_enc_val;
      u_m2_enc_val.real = this->m2_enc_val;
      *(outbuffer + offset + 0) = (u_m2_enc_val.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m2_enc_val.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m2_enc_val.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m2_enc_val.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m2_enc_val);
      union {
        int32_t real;
        uint32_t base;
      } u_m1_enc_qpps;
      u_m1_enc_qpps.real = this->m1_enc_qpps;
      *(outbuffer + offset + 0) = (u_m1_enc_qpps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m1_enc_qpps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m1_enc_qpps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m1_enc_qpps.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m1_enc_qpps);
      union {
        int32_t real;
        uint32_t base;
      } u_m2_enc_qpps;
      u_m2_enc_qpps.real = this->m2_enc_qpps;
      *(outbuffer + offset + 0) = (u_m2_enc_qpps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m2_enc_qpps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m2_enc_qpps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m2_enc_qpps.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m2_enc_qpps);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_m1_enc_val;
      u_m1_enc_val.base = 0;
      u_m1_enc_val.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m1_enc_val.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m1_enc_val.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m1_enc_val.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m1_enc_val = u_m1_enc_val.real;
      offset += sizeof(this->m1_enc_val);
      union {
        int32_t real;
        uint32_t base;
      } u_m2_enc_val;
      u_m2_enc_val.base = 0;
      u_m2_enc_val.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m2_enc_val.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m2_enc_val.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m2_enc_val.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m2_enc_val = u_m2_enc_val.real;
      offset += sizeof(this->m2_enc_val);
      union {
        int32_t real;
        uint32_t base;
      } u_m1_enc_qpps;
      u_m1_enc_qpps.base = 0;
      u_m1_enc_qpps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m1_enc_qpps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m1_enc_qpps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m1_enc_qpps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m1_enc_qpps = u_m1_enc_qpps.real;
      offset += sizeof(this->m1_enc_qpps);
      union {
        int32_t real;
        uint32_t base;
      } u_m2_enc_qpps;
      u_m2_enc_qpps.base = 0;
      u_m2_enc_qpps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m2_enc_qpps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m2_enc_qpps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m2_enc_qpps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m2_enc_qpps = u_m2_enc_qpps.real;
      offset += sizeof(this->m2_enc_qpps);
     return offset;
    }

    virtual const char * getType() override { return "roboclaw_driver/Stats"; };
    virtual const char * getMD5() override { return "210b25d19a34293aed84261a946d88a5"; };

  };

}
#endif
