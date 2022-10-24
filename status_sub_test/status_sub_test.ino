/*
   rosserial Subscriber Example
   Blinks an LED on callback
*/

#include <ros.h>
#include <azimorph_msg/Node.h>
#include <azimorph_msg/Topic.h>
#include <azimorph_msg/Status.h>

ros::NodeHandle  nh; //DONT COPY

struct  {
  String names;
  int freqs;
  int statuses;
} topic[] = {};

struct  {
  String names;
  int statuses;
} node[] = {};

void statusCallBack (const azimorph_msg::Status& data) {
  int error = 1;
  int topic_len = sizeof(data.Topics);
  int node_len = sizeof(data.Nodes);
  for (int i = 0; i < topic_len; i++) {
    topic[i].names = data.Topics[i].name;
    topic[i].freqs = data.Topics[i].freq;
    topic[i].statuses = data.Topics[i].status;
  }
  for (int i = 0; i < node_len; i++) {
    node[i].names = data.Nodes[i].name;
    node[i].statuses = data.Nodes[i].status;
  }
}

ros::Subscriber<azimorph_msg::Status> status_sub("/status", &statusCallBack );

void setup()
{
  nh.initNode();
  nh.subscribe(status_sub);
  Serial.begin(9600);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
