
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <ras_phidget/msg/motor_encoder.msg>
#include <math.h>



class MotorControllerNode
{
public:
    ros::NodeHandle n;
    ros::Publisher pub_motor_right;
    ros::Publisher pub_motor_left;
    ros::Subscriber sub_velocity;
    ros::Subscriber sub_encoder_left;
    ros::Subscriber sub_encoder_right;



    MotorControllerNode(){
        // constructor
        n = ros::NodeHandle("~"); //??

        vel_time = ros::Time::now();

        control_frequency = 10.0; //Hz
        wheel_radius = 0.097/2.0; //meters
        base = 0.209; //meters
        ticks_per_rev = 3591.84;

        desired_w_left = 0.0;
        desired_w_right = 0.0;

        actual_w_left = 0.0;
        actual_w_right = 0.0;

        delta_enc_left = 0.0;
        delta_enc_right = 0.0;

        K_p = 80.0;
        K_i = 2.0;

        int_error_left = 0.0;
        int_error_right = 0.0;


        pub_motor_left = n.advertise<std_msgs::Float32>("/left_motor/cmd_vel", 10);
        pub_motor_right = n.advertise<std_msgs::Float32>("/right_motor/cmd_vel", 10);

        sub_velocity = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist",10,&MotorControllerNode::velocityCallback,this);
        sub_encoder_left = n.subscribe("/left_motor/encoder",10,&MotorControllerNode::encoder_left_callback,this);
        sub_encoder_right = n.subscribe("/right_motor/encoder",10,&MotorControllerNode::encoder_right_callback,this);


    }

    ~MotorControllerNode(){

    }



    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
      //ROS_INFO("velocityCallback velocity linear: [%f] [%s] [%f]", msg->linear.x, ", angular: ", msg->angular.z);

      linear_x = (float) msg->linear.x;
      angular_z = (float) msg->angular.z;

      desired_w_left = linear_x - base*angular_z;
      desired_w_right = linear_x + base*angular_z;

      //desired_w_left = (linear_x-0.5*base*angular_z)/wheel_radius;
      //desired_w_right = (linear_x+0.5*base*angular_z)/wheel_radius;

    }

    void encoder_left_callback(const ras_phidget::motor_encoder& msg)
    {
        delta_enc_left = (float) msg.count_change;
    }

    void encoder_right_callback(const ras_phidget::motor_encoder& msg)
    {
        delta_enc_right = (float) msg.count_change;
    }

    float calculateWheelVelocity(delta_encoder){
        float velocity = (2*M_PI*wheel_radius*control_frequency*delta_encoder)/ticks_per_rev;
        return velocity;
    }


    float getControlFrequency()
    {
        return control_frequency;
    }

    float piControlLeft(error){

    }

    float piControlRight(error){
      
    }

    void controlMotor(){

      actual_w_left = calculateWheelVelocity(delta_enc_left);
      actual_w_right = calculateWheelVelocity(delta_enc_right);

      float error_left = desired_w_left - actual_w_left;
      float error_right = desired_w_right - actual_w_right;



      float diff_vel_left = piControlLeft(error_left);
      float diff_vel_right = piControlRight(error_right);

      vel_msg_left.data = ;
      vel_msg_right.data = -desired_w_right;

      pub_motor_left.publish(vel_msg_left);
      pub_motor_right.publish(vel_msg_right);

    }

private:


    std_msgs::Float32 vel_msg_left;
    std_msgs::Float32 vel_msg_right;
    ros::Time  vel_time;

    double control_frequency;

    float linear_x;
    float angular_z;
    float desired_w_left;
    float desired_w_right;

    float actual_w_left;
    float actual_w_right;

    float delta_enc_left;
    float delta_enc_right;

    float K_p;
    float K_i;
    float int_error_left;
    float int_error_right;

    float wheel_radius;
    float base;
    float ticks_per_rev;

};



int main(int argc, char **argv)
{
  double control_frequency = 10.0;

  ros::init(argc, argv, "motor_controller");
  MotorControllerNode motor_controller;


  ros::Rate loop_rate(motor_controller.getControlFrequency());


  while(ros::ok())
  {
      motor_controller.controlMotor();
      ros::spinOnce();
      loop_rate.sleep();
  }

  //ros::spin();

  return 0;
}
