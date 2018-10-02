
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


class CircleCartesianControllerNode
{
public:
    ros::NodeHandle n;
    ros::Publisher publisher_twist;



    CircleCartesianControllerNode(){
        // constructor
        n = ros::NodeHandle("~");

        // Complete r=0.5 circle in 10 s
        linear_velocity = 0.5; //1*(0.5*M_PI/10); // 2*pi*r/10 = 2*pi*0.5/10 m/s
        angular_velocity = M_PI/10; //1*(2*M_PI/10); // 2*pi meters in 10 s
        frequency = 10;

        twist_msg.linear.x = linear_velocity;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_velocity;

        publisher_twist = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);

    }

    void publishVelocity()
    {
        publisher_twist.publish(twist_msg);
    }


    float getFrequency()
    {
        return frequency;
    }


private:
    geometry_msgs::Twist twist_msg;
    double frequency;
    float linear_velocity;
    float angular_velocity;

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "circle_controller");
  CircleCartesianControllerNode circle_controller;

  ros::Rate loop_rate(circle_controller.getFrequency());

  while(ros::ok())
  {
      circle_controller.publishVelocity();
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
