#ifndef _ROS_status_node_Topic_h
#define _ROS_status_node_Topic_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace status_node
{

  class Topic : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef uint16_t _freq_type;
      _freq_type freq;
      typedef int8_t _status_type;
      _status_type status;

    Topic():
      name(""),
      freq(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->freq >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->freq >> (8 * 1)) & 0xFF;
      offset += sizeof(this->freq);
      union {
        int8_t real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      this->freq =  ((uint16_t) (*(inbuffer + offset)));
      this->freq |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->freq);
      union {
        int8_t real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    virtual const char * getType() override { return "status_node/Topic"; };
    virtual const char * getMD5() override { return "1dea693c77c88410953b7037629a5ed8"; };

  };

}
#endif
