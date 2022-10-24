#ifndef _ROS_status_node_Status_h
#define _ROS_status_node_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "status_node/Topic.h"
#include "status_node/Node.h"

namespace status_node
{

  class Status : public ros::Msg
  {
    public:
      uint32_t Topics_length;
      typedef status_node::Topic _Topics_type;
      _Topics_type st_Topics;
      _Topics_type * Topics;
      uint32_t Nodes_length;
      typedef status_node::Node _Nodes_type;
      _Nodes_type st_Nodes;
      _Nodes_type * Nodes;

    Status():
      Topics_length(0), st_Topics(), Topics(nullptr),
      Nodes_length(0), st_Nodes(), Nodes(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->Topics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Topics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Topics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Topics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Topics_length);
      for( uint32_t i = 0; i < Topics_length; i++){
      offset += this->Topics[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->Nodes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Nodes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Nodes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Nodes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Nodes_length);
      for( uint32_t i = 0; i < Nodes_length; i++){
      offset += this->Nodes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t Topics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      Topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      Topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      Topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->Topics_length);
      if(Topics_lengthT > Topics_length)
        this->Topics = (status_node::Topic*)realloc(this->Topics, Topics_lengthT * sizeof(status_node::Topic));
      Topics_length = Topics_lengthT;
      for( uint32_t i = 0; i < Topics_length; i++){
      offset += this->st_Topics.deserialize(inbuffer + offset);
        memcpy( &(this->Topics[i]), &(this->st_Topics), sizeof(status_node::Topic));
      }
      uint32_t Nodes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      Nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      Nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      Nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->Nodes_length);
      if(Nodes_lengthT > Nodes_length)
        this->Nodes = (status_node::Node*)realloc(this->Nodes, Nodes_lengthT * sizeof(status_node::Node));
      Nodes_length = Nodes_lengthT;
      for( uint32_t i = 0; i < Nodes_length; i++){
      offset += this->st_Nodes.deserialize(inbuffer + offset);
        memcpy( &(this->Nodes[i]), &(this->st_Nodes), sizeof(status_node::Node));
      }
     return offset;
    }

    virtual const char * getType() override { return "status_node/Status"; };
    virtual const char * getMD5() override { return "192769bdd9c6a520f3f737b99d602547"; };

  };

}
#endif
