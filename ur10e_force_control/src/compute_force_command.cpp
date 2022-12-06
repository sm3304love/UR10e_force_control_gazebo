#include "ros/ros.h"
#include "ur10e_force_msgs/ForceCmd.h"
#include <cstdlib>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "force_command_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ur10e_force_msgs::ForceCmd>("force_cmd");
  ur10e_force_msgs::ForceCmd srv;
  
  if(argc != 9) // 다시한번 확인
  {
    for(int i = 0; i<6; i++)
    {
        srv.request.TargetTheta[i] = atof(argv[i+1]);
    }
    srv.request.Tf = atof(argv[7]);
   

    ROS_INFO("Target theta = %f,%f,%f,%f,%f,%f ",srv.request.TargetTheta[0],srv.request.TargetTheta[1],srv.request.TargetTheta[2],srv.request.TargetTheta[3],srv.request.TargetTheta[4],srv.request.TargetTheta[5] );
    ROS_INFO("Target Tf = %f", srv.request.Tf);
    if(client.call(srv))
    {
        if(srv.response.FinishFlag == 1)
        {
            ROS_INFO("Finish");
        }
        else
            ROS_ERROR("Calculating");
    }
  }
  else
    ROS_ERROR("Too many arguments");
  
  return 0;
}