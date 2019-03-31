#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"cmd_vel_demo");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist msg;
    // for(int i=0;i<=1;i++)
    // {
    //     msg.linear.x=0.5;
    //     msg.linear.y=1;
    //     msg.linear.z=0;
    //     msg.angular.x=0;
    //     msg.angular.y=0;
    //     msg.angular.z=0;
    //     pub.publish(msg);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    while(ros::ok()){
        msg.linear.x=0.5;
        msg.linear.y=1;
        msg.linear.z=0;
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}