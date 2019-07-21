#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <tf/tf.h>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>
#define pi 3.14159265

float radius = 1,rad = -2*pi/60;
ros::Subscriber vicon_sub;
double yaw,x,y;
double ref_yaw;
double Xc,Yc,rightw,leftw;
ros::Publisher vel_pub,ref_pub;
float r = 0.05;
float D = 0.05;
float L = 0.05;
geometry_msgs::PoseStamped reftraj;
ros::Time last_received;
std_msgs::Int16MultiArray twist;
int complete_flag = 0;


double Xt(double time)
{
  //return 1;
  return radius*cos(time*rad);
}

double Yt(double time)
{
  //return 0;
  return radius*sin(time*rad);
}

double dXt(double time)
{
  //return 0;
  return -rad*radius*sin(time*rad);
}

double dYt(double time)
{
  //return 0;
  return rad*radius*cos(time*rad);
}

/*void viconCallback(vicon::Subject rover)
{
  x = rover.position.x;
  y = rover.position.y;

  tf::Quaternion q(rover.orientation.x,rover.orientation.y,rover.orientation.z,rover.orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch;
  m.getRPY(roll,pitch,yaw);
}*/

void viconCallback(geometry_msgs::PoseStamped rover)
{
  x = rover.pose.position.x - 1;
  y = rover.pose.position.y;
  last_received = ros::Time::now();
   tf::Quaternion q(rover.pose.orientation.x,rover.pose.orientation.y,rover.pose.orientation.z,rover.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch;
  m.getRPY(roll,pitch,yaw);
  yaw = yaw - pi/2;
  if (yaw>pi) yaw-= 2*pi;
  if (yaw<-pi) yaw+= 2*pi;

  if (yaw*180/pi <10 && yaw*180/pi>-10)
    complete_flag = 1;
  if (complete_flag==1 && yaw*180/pi >170 && yaw*180/pi<180)
    {
      complete_flag = 2;
      ROS_WARN("Mission Complete");
    }
  ref_yaw = atan2(y,x);
  double yaw_correction = ref_yaw - yaw;
  if (yaw_correction>pi)
    yaw_correction -= 2*pi;
  if (yaw_correction<-pi)
    yaw_correction+= 2*pi;
std::cout<<"X: "<<x<<" Y: "<<y<<" Yaw: "<<yaw*180/pi<<" Ref Yaw: "<<ref_yaw*180/pi<<" Error: "<<yaw_correction<<std::endl;
if (yaw_correction<0.05)
{
  leftw = 90;
  rightw = 70;
}
else
{
leftw = 70 - yaw_correction*15;
rightw = 70 + yaw_correction*15;
}
if (leftw>150) leftw = 150;
    if (leftw<-150) leftw = -150;
    if (rightw>150) rightw = 150;
    if (rightw<-150) rightw = -150;
    
    if(complete_flag==2)
    {
    leftw = 0;
    rightw = 0;
  }
twist.data.clear();
    twist.data.push_back(leftw);
    twist.data.push_back(rightw);
    twist.data.push_back(0);
    twist.data.push_back(0);
    
    //Publish necessary topics
    vel_pub.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle nh;
  //Server as(nh, "Gollum", boost::bind(&executeCB, _1, &as), false);
  int freq = 10;
  ros::Rate loop_rate(freq);
  last_received = ros::Time::now();
  ros::Duration d;
  //vicon_sub = nh.subscribe("/vicon/Gollum", 100, viconCallback);
  vicon_sub = nh.subscribe("/Gollum/RGBD/CamPoseCamFrame", 100, viconCallback);
  vel_pub = nh.advertise<std_msgs::Int16MultiArray>("/cmd",10);
  ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/ref_traj",10);
  double time;
  
  float rho = D/L;
  double J11,J12,J21,J22;
  reftraj.header.frame_id = "slam";
  double Iyerr=0.0,Ixerr=0.0,Dxerr=0.0,Dyerr=0.0,OldErrX=0.0,OldErrY=0.0;
  while(ros::ok()){
    d = ros::Time::now() - last_received;
    time = d.toSec();
    if(time>3)
    {
      twist.data.clear();
    twist.data.push_back(0);
    twist.data.push_back(0);
    twist.data.push_back(0);
    twist.data.push_back(0);
    
    //Publish necessary topics
    vel_pub.publish(twist);
    }
    
    //Make the Jinv matrix
    J11 = (1/r)*(cos(yaw)+sin(yaw)/rho);
    J12 = (1/r)*(sin(yaw)-cos(yaw)/rho);
    J21 = (1/r)*(cos(yaw)-sin(yaw)/rho);
    J22 = (1/r)*(sin(yaw)+cos(yaw)/rho);
    
    //Publish a reference trajectory to monitor on rviz
    reftraj.pose.position.x = Xt(time);//+cos(yaw)*0.05;
    reftraj.pose.position.y = Yt(time);//+sin(yaw)*0.05;
    reftraj.pose.position.z = 0;
    reftraj.pose.orientation.w = cos(time*rad/2.0 - pi/4);
    reftraj.pose.orientation.z = sin(time*rad/2.0 - pi/4);
    /*
    Ixerr += (Xt(time) - x)/freq;
    Iyerr += (Yt(time) - y)/freq;
    if (Ixerr>0.5)
      Ixerr = 0.5;
    if (Iyerr>0.5)
      Iyerr =0.5;
    Dxerr = Xt(time) - x - OldErrX;
    Dxerr = Yt(time) - y - OldErrY;
    OldErrX = Xt(time) - x;
    OldErrY = Yt(time) - y;
    //Calculate wheel speeds
    Xc = dXt(time) + Kp*(Xt(time) - x) + Ki*Ixerr - Kd*Dxerr;
    Yc = dYt(time) + Kp*(Yt(time) - y) + Ki*Iyerr - Kd*Dyerr;
    //std::cout<<"Trying to get to "<<Xc<<" "<<Yc<<std::endl;
    leftw = -(J11*Xc + J12*Yc);
    rightw = -(J21*Xc + J22*Yc);
    
    //Saturate wheel speeds and push it into the topic type
    leftw = 100;
    rightw = 100;

    if (leftw>100) leftw = 100;
    if (leftw<-100) leftw = -100;
    if (rightw>100) rightw = 100;
    if (rightw<-100) rightw = -100;
    
    twist.data.clear();
    twist.data.push_back(leftw);
    twist.data.push_back(rightw);
    twist.data.push_back(0);
    twist.data.push_back(0);
    
    //Publish necessary topics
    vel_pub.publish(twist);
    ref_pub.publish(reftraj);*/
    ros::spinOnce();
    loop_rate.sleep();
  }
  
}
