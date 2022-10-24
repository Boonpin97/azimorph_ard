#ifndef _ROS_roboclaw_driver_SpeedCommand_h
#define _ROS_roboclaw_driver_SpeedCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roboclaw_driver
{

  class SpeedCommand : public ros::Msg
  {
    public:
      typedef int32_t _m1_qpps_type;
      _m1_qpps_type m1_qpps;
      typedef int32_t _m2_qpps_type;
      _m2_qpps_type m2_qpps;
      typedef uint32_t _accel_type;
      _accel_type accel;
      typedef uint32_t _max_secs_type;
      _max_secs_type max_secs;

    SpeedCommand():
      m1_qpps(0),
      m2_qpps(0),
      accel(0),
      max_secs(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_m1_qpps;
      u_m1_qpps.real = this->m1_qpps;
      *(outbuffer + offset + 0) = (u_m1_qpps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m1_qpps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m1_qpps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m1_qpps.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m1_qpps);
      union {
        int32_t real;
        uint32_t base;
      } u_m2_qpps;
      u_m2_qpps.real = this->m2_qpps;
      *(outbuffer + offset + 0) = (u_m2_qpps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m2_qpps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m2_qpps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m2_qpps.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m2_qpps);
      *(outbuffer + offset + 0) = (this->accel >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accel >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accel >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accel >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel);
      *(outbuffer + offset + 0) = (this->max_secs >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_secs >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_secs >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_secs >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_secs);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_m1_qpps;
      u_m1_qpps.base = 0;
      u_m1_qpps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m1_qpps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m1_qpps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m1_qpps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m1_qpps = u_m1_qpps.real;
      offset += sizeof(this->m1_qpps);
      union {
        int32_t real;
        uint32_t base;
      } u_m2_qpps;
      u_m2_qpps.base = 0;
      u_m2_qpps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m2_qpps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m2_qpps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m2_qpps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m2_qpps = u_m2_qpps.real;
      offset += sizeof(this->m2_qpps);
      this->accel =  ((uint32_t) (*(inbuffer + offset)));
      this->accel |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accel |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->accel |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->accel);
      this->max_secs =  ((uint32_t) (*(inbuffer + offset)));
      this->max_secs |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->max_secs |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->max_secs |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->max_secs);
     return offset;
    }

    virtual const char * getType() override { return "roboclaw_driver/SpeedCommand"; };
    virtual const char * getMD5() override { return "1217b4f680d01eb5008f99442806c05f"; };

  };

}
#endif
