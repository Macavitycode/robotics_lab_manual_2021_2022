#include <ros.h>
#include <std_srvs/SetBool.h>

ros::NodeHandle nh;

void callback(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{

  digitalWrite(13, HIGH-digitalRead(13));
  
}

ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> server("blink", &callback);

void setup() {
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertiseService(server);

}

void loop() 
{

  nh.spinOnce();
  delay(1);
  
}
