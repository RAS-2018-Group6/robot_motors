
#include <ros/ros.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Pose.h>
#include <math.h>


class DeadReckoningNode
{
public:
    ros::NodeHandle n;
    ros::Publisher pub_odom;
    ros::Subscriber sub_encoder_left;
    ros::Subscriber sub_encoder_right;

    DeadReckoningNode(){
        n = ros::NodeHandle("~"); //????

        msg_time = ros::Time::now();

        ticks_per_rev = 890.0;
        control_frequency = 10.0; //Hz
        wheel_radius = 0.097/2.0; //meters
        base = 0.209;

        x = 0;
        y = 0;
        phi = 0;

        delta_x = 0;
        delta_y = 0;
        delta_phi = 0;

        dt = 0;

        actual_w_left = 0;
        actual_w_right = 0;

        delta_enc_left = 0;
        delta_enc_right = 0;

        prev_enc_left = 0;
        prev_enc_right = 0;

        // 465093
        // 465084
        pub_odom = n.advertise<geometry_msgs::Pose>("/odom", 5);
        sub_encoder_left = n.subscribe<phidgets::motor_encoder>("/left_motor/encoder",10,&DeadReckoningNode::encoder_left_callback,this);
        sub_encoder_right = n.subscribe<phidgets::motor_encoder>("/right_motor/encoder",10,&DeadReckoningNode::encoder_right_callback,this);
    }

    ~DeadReckoningNode(){

    }


    void encoder_left_callback(const phidgets::motor_encoder::ConstPtr& msg)
    {
        //delta_enc_left = (float) msg->count_change;
        count_enc_left = (float) msg->count;
        delta_enc_left = count_enc_left-prev_enc_left;
        prev_enc_left = count_enc_left;
    }

    void encoder_right_callback(const phidgets::motor_encoder::ConstPtr& msg)
    {
        //delta_enc_right = - (float) msg->count_change;
        count_enc_right = -(float) msg->count;
        delta_enc_right = count_enc_right-prev_enc_right;
        prev_enc_right = count_enc_right;
        //ROS_INFO("delta encoder_right: %f",delta_enc_right);
    }

    void calculatePosition()
    {
        float actual_w_left = (delta_enc_left*2*M_PI*control_frequency)/ticks_per_rev;
        float actual_w_right= (delta_enc_right*2*M_PI*control_frequency)/ticks_per_rev;

        dt = (ros::Time::now() - msg_time).toSec();

        delta_x = 0.5*wheel_radius*cos(phi)*(actual_w_left+actual_w_right);
        delta_y = 0.5*wheel_radius*sin(phi)*(actual_w_right+actual_w_right);

        delta_phi = 0.5*(-actual_w_left+actual_w_right)/base;
        phi = (phi+delta_phi*dt);

        x = x+delta_x*dt;
        y = y+delta_y*dt;

        msg_time = ros::Time::now();

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
    ros::Time msg_time;

    double control_frequency;
    float ticks_per_rev;
    float wheel_radius;
    float base;

    float actual_w_left;
    float actual_w_right;

    float delta_enc_left;
    float delta_enc_right;

    float count_enc_left;
    float count_enc_right;

    float prev_enc_left;
    float prev_enc_right;

    float delta_x;
    float delta_y;
    float delta_phi;

    float x;
    float y;
    float phi;
    float dt;

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

  return 0;
}
