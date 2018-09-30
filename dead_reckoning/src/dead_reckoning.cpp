
#include <ros/ros.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Pose.h>
#include <math.h>



class DeadReckoningNode
{
public:
    ros::NodeHandle n;
    ros::Publisher pub_odom;
    ros::Subscriber subscriber_encoders;



    DeadReckoningNode(){
        // constructor
        n = ros::NodeHandle("~"); //????

        ticks_per_rev = 360;
        control_frequency = 10.0; //Hz
        wheel_radius = 0.0352; //meters
        base = 0.23;

        x = 0;
        y = 0;
        phi = 0;

        delta_x = 0;
        delta_y = 0;
        delta_phi = 0;

        dt = 1/control_frequency;

        d_encoder1 = 0;
        d_encoder2 = 0;

        // 465093
        // 465084
        pub_odom = n.advertise<geometry_msgs::Pose>("/odom", 1);
        sub_encoder1 = n.subscribe<phidgets::motor_encoder>("/left_motor/encoder",10,&DeadReckoningNode::encoder1Callback,this);
        sub_encoder2 = n.subscribe<phidget::motor_encoder>("/right_motor/encoder",10,&DeadReckoningNode::encoder2Callback,this);


    }



    void encoder1Callback(const phidgets::motor_encoder::ConstPtr& msg)
    {
      //ROS_INFO("I heard encoder1: [%d] %s [%d]", msg->delta_encoder1,", encoder2: ", msg->delta_encoder2);

      encoder1 = msg->count;
      d_encoder1 = msg->count_change;

    }


    void encoder2Callback(const phidgets::motor_encoder::ConstPtr& msg)
    {
      //ROS_INFO("I heard encoder1: [%d] %s [%d]", msg->delta_encoder1,", encoder2: ", msg->delta_encoder2);

      encoder2 = msg->count;
      d_encoder2 = msg->count_change;

    }





    void calculatePosition()
    {
        float float_d_encoder1 = (float)(d_encoder1);
        float float_d_encoder2 = (float)(d_encoder2);

        float estimated_w1 = (float_d_encoder1*2*M_PI*control_frequency)/ticks_per_rev;
        float estimated_w2 = (float_d_encoder2*2*M_PI*control_frequency)/ticks_per_rev;

        delta_phi = wheel_radius*(-estimated_w1+estimated_w2)/base;
        phi = (phi+delta_phi*dt);


        delta_x = -0.5*wheel_radius*sin(phi)*(estimated_w1+estimated_w2);
        delta_y = 0.5*wheel_radius*cos(phi)*(estimated_w1+estimated_w2);

        x = x+delta_x*dt;
        y = y+delta_y*dt;


        // --------------------------------------------------------------

        odom_msg.position.x = x;
        odom_msg.position.y = y;
        odom_msg.orientation.z = phi;

        pub_odom.publish(odom_msg);


        ROS_INFO("Position (x,y,phi) = (%f,%f,%f)", x,y,phi);

    }


    void positioningLoop()
    {

        calculatePosition();
    }


    float getControlFrequency()
    {
        return control_frequency;
    }


private:
    geometry_msgs::Pose odom_msg;

    double control_frequency;
    int ticks_per_rev;
    int encoder1;
    int encoder2;
    int d_encoder1;
    int d_encoder2;


    float delta_x;
    float delta_y;
    float delta_phi;

    float x;
    float y;
    float phi;
    float dt;


    float wheel_radius;
    float base;

};



int main(int argc, char **argv)
{

  ros::init(argc, argv,"dead_reckoning");
  DeadReckoningNode dead_reckoning;


  ros::Rate loop_rate(dead_reckoning.getControlFrequency());


  while(ros::ok())
  {
      dead_reckoning.positioningLoop();

      ros::spinOnce();
      loop_rate.sleep();
  }

  ros::spin();

  return 0;
}

