
#include <ur10e_force_control/parameter.h>
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "ur10e_force_msgs/ForceCmd.h"
#include <sstream>
#include "math.h"

Eigen::VectorXd ComputedTorque_Ftip(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& eint,
		const Eigen::VectorXd& g, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist,
		const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetalistd, const Eigen::VectorXd& dthetalistd, const Eigen::VectorXd& ddthetalistd,
		const Eigen::VectorXd& Ftip , double Kp, double Ki, double Kd);

Eigen::MatrixXd MassMatrix(const Eigen::VectorXd& thetalist,
                                const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist);

#define join_size 6 //num of joint

class force_control_temp
{
public:
  force_control_temp()
  {
    service = n.advertiseService("force_cmd", &force_control_temp::test,this);
    joint_state_sub = n.subscribe("/ur10e/joint_states", 1000, &force_control_temp::urCallback,this);
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
    thetalist << JointPosition[0],JointPosition[1],JointPosition[2],JointPosition[3],JointPosition[4],JointPosition[5];
    dthetalist << JointVelocity[0],JointVelocity[1],JointVelocity[2],JointVelocity[3],JointVelocity[4],JointVelocity[5];

  }

  bool test(ur10e_force_msgs::ForceCmd::Request  &req, ur10e_force_msgs::ForceCmd::Response &res)
  {
    for(int i = 0; i<6; i++)
      {
          ThetaCmd[i] = req.TargetTheta[i];
      }

    Tf = req.Tf;
  
    thetaend << ThetaCmd[0], ThetaCmd[1],ThetaCmd[2],ThetaCmd[3],ThetaCmd[4],ThetaCmd[5];
    flag = true;
    ROS_WARN_STREAM("f");
    res.FinishFlag = flag;
    
    return true;
  }

  void run()
  {
    while(ros::ok)
    {
        thetalistd = thetaend;
        if(flag == false)
        {
          ros::Rate loop_rate(1000);
          Eigen::VectorXd grav = ComputedTorque_Ftip(thetalist, dthetalist, eint, gravity, Mlist, Glist, Slist, thetalistd, dthetalistd, ddthetalistd, Ftip, Kp, Ki, Kd);
          effort1.data = grav(0);
          effort2.data = grav(1);
          effort3.data = grav(2);
          effort4.data = grav(3);
          effort5.data = grav(4);
          effort6.data = grav(5);
          shoulder_pan_joint_effort_pub.publish(effort1);
          shoulder_lift_joint_effort_pub.publish(effort2);
          elbow_joint_effort_pub.publish(effort3);
          wrist_1_joint_effort_pub.publish(effort4);
          wrist_2_joint_effort_pub.publish(effort5);
          wrist_3_joint_effort_pub.publish(effort6); 
          ROS_WARN_STREAM(effort1.data << ", " << effort2.data << ", " << effort3.data << ", " << effort4.data << ", " << effort5.data << ", " << effort6.data);

          loop_rate.sleep();
        }
        else if(flag == true)
        {
          
          int N = int(1.0*Tf / dt);
          Eigen::MatrixXd traj = mr::JointTrajectory(thetalist, thetaend, Tf, N, method);
          Eigen::MatrixXd thetamatd = traj;
          Eigen::MatrixXd dthetamatd = Eigen::MatrixXd::Zero(N, 6);
          Eigen::MatrixXd ddthetamatd = Eigen::MatrixXd::Zero(N, 6);
          
          dt = Tf / (N - 1.0);
          hz = 1/ dt;
          ros::Rate loop_rate(hz);
          for (int i = 0; i < N - 1; ++i) {
            dthetamatd.row(i + 1) = (thetamatd.row(i + 1) - thetamatd.row(i)) / dt;
            ddthetamatd.row(i + 1) = (dthetamatd.row(i + 1) - dthetamatd.row(i)) / dt;
          }
         
        
          for( int i = 0; i < thetamatd.rows(); i++) //계산 속도 느림 -> trajectory 수 변경?
          {            
            thetalistd = thetamatd.row(i);
            dthetalistd = dthetamatd.row(i);
            ddthetalistd = ddthetamatd.row(i);
            Eigen::VectorXd torque = ComputedTorque_Ftip(thetalist, dthetalist, eint, gravity, Mlist, Glist, Slist, thetalistd, dthetalistd, ddthetalistd, Ftip, Kp, Ki, Kd);
            effort1.data = torque(0);
            effort2.data = torque(1);
            effort3.data = torque(2);
            effort4.data = torque(3);
            effort5.data = torque(4);
            effort6.data = torque(5);
            shoulder_pan_joint_effort_pub.publish(effort1);
            shoulder_lift_joint_effort_pub.publish(effort2);
            elbow_joint_effort_pub.publish(effort3);
            wrist_1_joint_effort_pub.publish(effort4);
            wrist_2_joint_effort_pub.publish(effort5);
            wrist_3_joint_effort_pub.publish(effort6); 
            loop_rate.sleep();
            ROS_WARN_STREAM(effort1.data << ", " << effort2.data << ", " << effort3.data << ", " << effort4.data << ", " << effort5.data << ", " << effort6.data);
  
          }
          flag = false;
        }
 
    }
  }

private:
  ros::NodeHandle n;
  ros::ServiceServer service;
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
  
  double JointPosition[join_size] = {0.0};
  double JointVelocity[join_size] = {0.0};
  double JointTarget[join_size] = {0,0,0,0,0,0};
  double ThetaCmd[6] = {0.0};
  bool flag = false;
  double Tf = 0.0;
  double dt = 0.01;
  int method = 5;
  bool done = false;

  Eigen::VectorXd thetaend = Eigen::VectorXd::Zero(6);  

  Eigen::VectorXd eint = Eigen::VectorXd::Zero(6);

  Eigen::VectorXd thetalist = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd dthetalist = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(6);

  Eigen::VectorXd thetalistd = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd dthetalistd = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd ddthetalistd = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Ftip = Eigen::VectorXd::Zero(6);
  
  double Kp= 20;
  double Ki= 0;
  double Kd= 15;


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "force_command_server");
  ROS_INFO("Ready to service.");
  ros::AsyncSpinner spinner(10);
  spinner.start();

  setup();
  force_control_temp force_node;
  force_node.run();

  return 0;
}


// 연산이 느림 -> cpu 성능에 따라 달라짐
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



