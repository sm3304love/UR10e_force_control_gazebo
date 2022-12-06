#include <ur10e_force_control/parameter.h>
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "ur10e_force_msgs/ForceCmd.h"
#include <sstream>
#include "math.h"

#define join_size 6 //num of joint

Eigen::VectorXd ComputedTorque_Ftip(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& eint,
		const Eigen::VectorXd& g, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist,
		const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetalistd, const Eigen::VectorXd& dthetalistd, const Eigen::VectorXd& ddthetalistd,
		const Eigen::VectorXd& Ftip , double Kp, double Ki, double Kd);


class force_control_client_subscriber
{
public:
  force_control_client_subscriber()
  {
    joint_state_sub = n.subscribe("/ur10e/joint_states", 100, &force_control_client_subscriber::urCallback,this);
    elbow_joint_effort_pub = n.advertise<std_msgs::Float64>("/ur10e/elbow_joint_effort_controller/command", 100);
    shoulder_lift_joint_effort_pub = n.advertise<std_msgs::Float64>("/ur10e/shoulder_lift_joint_effort_controller/command", 100);
    shoulder_pan_joint_effort_pub = n.advertise<std_msgs::Float64>("/ur10e/shoulder_pan_joint_effort_controller/command", 100);
    wrist_1_joint_effort_pub = n.advertise<std_msgs::Float64>("/ur10e/wrist_1_joint_effort_controller/command", 100);
    wrist_2_joint_effort_pub = n.advertise<std_msgs::Float64>("/ur10e/wrist_2_joint_effort_controller/command", 100);
    wrist_3_joint_effort_pub = n.advertise<std_msgs::Float64>("/ur10e/wrist_3_joint_effort_controller/command", 100);
  }
  
  void urCallback(const sensor_msgs::JointState::ConstPtr& JointState)
  {
     // ur10e/joint_states value order is messed up
    JointPosition[0] = JointState->position[2];
    JointPosition[1] = JointState->position[1];
    JointPosition[2] = JointState->position[0];
    JointPosition[3] = JointState->position[3];
    JointPosition[4] = JointState->position[4];
    JointPosition[5] = JointState->position[5];

    JointVelocity[0] = JointState->velocity[2];
    JointVelocity[1] = JointState->velocity[1];
    JointVelocity[2] = JointState->velocity[0];
    JointVelocity[3] = JointState->velocity[3];
    JointVelocity[4] = JointState->velocity[4];
    JointVelocity[5] = JointState->velocity[5];

  }



  void run(){
    ros::Rate loop_rate(1000);
    while(ros::ok)
    {
      thetalist << JointPosition[0],JointPosition[1],JointPosition[2],JointPosition[3],JointPosition[4],JointPosition[5];
      dthetalist << JointVelocity[0],JointVelocity[1],JointVelocity[2],JointVelocity[3],JointVelocity[4],JointVelocity[5];
 
      Eigen::VectorXd taulist = ComputedTorque_Ftip(thetalist, dthetalist, eint, gravity, Mlist, Glist, Slist, thetalistd, dthetalistd, ddthetalistd, Ftip, Kp, Ki, Kd);
      effort1.data = taulist(0);
      effort2.data = taulist(1);
      effort3.data = taulist(2);
      effort4.data = taulist(3);
      effort5.data = taulist(4);
      effort6.data = taulist(5);
      shoulder_pan_joint_effort_pub.publish(effort1);
      shoulder_lift_joint_effort_pub.publish(effort2);
      elbow_joint_effort_pub.publish(effort3);
      wrist_1_joint_effort_pub.publish(effort4);
      wrist_2_joint_effort_pub.publish(effort5);
      wrist_3_joint_effort_pub.publish(effort6); 
      ROS_WARN_STREAM(effort1.data << ", " << effort2.data << ", " << effort3.data << ", " << effort4.data << ", " << effort5.data << ", " << effort6.data);
      loop_rate.sleep();

    }
    
  }


private:
  
  float JointPosition[join_size] = {0.0};
  float JointVelocity[join_size] = {0.0};
  float JointTarget[join_size] = {0,0,0,0,0,0};


  ros::NodeHandle n;
  ros::Subscriber joint_state_sub;
  ros::Publisher elbow_joint_effort_pub;
  ros::Publisher shoulder_lift_joint_effort_pub;
  ros::Publisher shoulder_pan_joint_effort_pub;
  ros::Publisher wrist_1_joint_effort_pub;
  ros::Publisher wrist_2_joint_effort_pub;
  ros::Publisher wrist_3_joint_effort_pub;
  

  std_msgs::Float64 effort1;
  std_msgs::Float64 effort2;
  std_msgs::Float64 effort3;
  std_msgs::Float64 effort4;
  std_msgs::Float64 effort5;
  std_msgs::Float64 effort6;
  bool flag = false;
  double Tf = 0.0;
  int method = 5;


  Eigen::VectorXd eint = Eigen::VectorXd::Zero(6);

  Eigen::VectorXd thetalist = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd dthetalist = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(6);

  Eigen::VectorXd thetalistd = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd dthetalistd = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd ddthetalistd = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Ftip = Eigen::VectorXd::Zero(6);

  double Kp= 10;
  double Ki= 0;
  double Kd= 5;

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur10e_force_control_client");
  ros::AsyncSpinner spinner(10);
  spinner.start();

  setup();
  force_control_client_subscriber force_node;
  force_node.run();
  
  // ros::spin();
  return 0;
}


Eigen::VectorXd ComputedTorque_Ftip(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& eint,
		const Eigen::VectorXd& g, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist,
		const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetalistd, const Eigen::VectorXd& dthetalistd, const Eigen::VectorXd& ddthetalistd,
		const Eigen::VectorXd& Ftip , double Kp, double Ki, double Kd) {

		Eigen::VectorXd e = thetalistd - thetalist;  // position err
		Eigen::VectorXd tau_feedforward = mr::MassMatrix(thetalist, Mlist, Glist, Slist)*(Kp*e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist));
		Eigen::VectorXd tau_inversedyn = mr::InverseDynamics(thetalist, dthetalist, ddthetalistd, g, Ftip, Mlist, Glist, Slist);

		Eigen::VectorXd tau_computed = tau_feedforward + tau_inversedyn;
		return tau_computed;
              }
