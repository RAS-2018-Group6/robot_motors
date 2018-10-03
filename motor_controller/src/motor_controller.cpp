
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>



class MotorControllerNode
{
public:
    ros::NodeHandle n;
    ros::Publisher pub_motor_2;
    ros::Publisher pub_motor_1;
    ros::Subscriber sub_velocity;



    MotorControllerNode(){
        // constructor
        n = ros::NodeHandle("~"); //??


        control_frequency = 10.0; //Hz
        wheel_radius = 0.097/2.0; //meters
        base = 0.209; //meters

        desired_w1 = 0;
        desired_w2 = 0;


        pub_motor_1 = n.advertise<std_msgs::Float32>("/left_motor/cmd_vel", 10);
        pub_motor_2 = n.advertise<std_msgs::Float32>("/right_motor/cmd_vel", 10);

        sub_velocity = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist",10,&MotorControllerNode::velocityCallback,this);


    }

    ~MotorControllerNode(){

    }



    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
      //ROS_INFO("velocityCallback velocity linear: [%f] [%s] [%f]", msg->linear.x, ", angular: ", msg->angular.z);

      linear_x = msg->linear.x;
      angular_z = msg->angular.z;



      desired_w1 = (linear_x-0.5*base*angular_z)/wheel_radius;
      desired_w2 = (linear_x+0.5*base*angular_z)/wheel_radius;

      vel_msg1.data = desired_w1;
      vel_msg2.data = -desired_w2;

      pub_motor_1.publish(vel_msg1);
      pub_motor_2.publish(vel_msg2);

    }




    float getControlFrequency()
    {
        return control_frequency;
    }


private:


    std_msgs::Float32 vel_msg1;
    std_msgs::Float32 vel_msg2;

    double control_frequency;

    float linear_x;
    float angular_z;
    float desired_w1;
    float desired_w2;

    float wheel_radius;
    float base;

};



int main(int argc, char **argv)
{
  double control_frequency = 10.0;

  ros::init(argc, argv, "motor_controller");
  MotorControllerNode motor_controller;


  ros::Rate loop_rate(motor_controller.getControlFrequency());


  while(ros::ok())
  {

      ros::spinOnce();
      loop_rate.sleep();
  }

  //ros::spin();

  return 0;
}
