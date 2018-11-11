
#include <ros/ros.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


class DeadReckoningNode
{
public:
    ros::NodeHandle n;
    ros::Publisher pub_odom;
    ros::Subscriber sub_encoder_left;
    ros::Subscriber sub_encoder_right;
    tf::TransformBroadcaster odom_broadcaster;

    DeadReckoningNode(){
        n = ros::NodeHandle("~");

        first_msg_left = true;
        first_msg_right = true;

        last_msg_time = ros::Time::now();

        ticks_per_rev = 890.0;
        control_frequency = 25.0; //Hz
        wheel_radius = 0.097/2.0; //meters
        base = 0.18;

        x = 0.20;
        y = 0.30;
        phi = M_PI/2;

 	      //x = 0.00;
        //y = 0.00;
        //phi = 0;

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
        linear_vel = 0.0;
        angular_vel = 0.0;

        // 465093
        // 465084
        pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 1);
        sub_encoder_left = n.subscribe<phidgets::motor_encoder>("/left_motor/encoder",1,&DeadReckoningNode::encoder_left_callback,this);
        sub_encoder_right = n.subscribe<phidgets::motor_encoder>("/right_motor/encoder",1,&DeadReckoningNode::encoder_right_callback,this);
    }

    ~DeadReckoningNode(){

    }


    void encoder_left_callback(const phidgets::motor_encoder::ConstPtr& msg)
    {
        if(first_msg_left){
          first_msg_left = false;
          count_enc_left = (float) msg->count;
          last_msg_time_left = msg->header.stamp;

        }else{
          dt_left = (msg->header.stamp - last_msg_time_left).toSec();
          last_msg_time_left = msg->header.stamp;
          count_enc_left = (float) msg->count;
          delta_enc_left = count_enc_left-prev_enc_left;
          prev_enc_left = count_enc_left;
          actual_w_left = (delta_enc_left*2*M_PI*wheel_radius)/(dt_left*ticks_per_rev);
        }


    }

    void encoder_right_callback(const phidgets::motor_encoder::ConstPtr& msg)
    {
        if(first_msg_right){
          first_msg_right = false;
          count_enc_right = (float) msg->count;
          last_msg_time_right = msg->header.stamp;
        }else{
          dt_right = (msg->header.stamp-last_msg_time_right).toSec();
          last_msg_time_right = msg->header.stamp;
          count_enc_right = -(float) msg->count;
          delta_enc_right = count_enc_right-prev_enc_right;
          prev_enc_right = count_enc_right;
          actual_w_right= (delta_enc_right*2*M_PI*wheel_radius)/(dt_right*ticks_per_rev);
        }


        //ROS_INFO("delta encoder_right: %f",delta_enc_right);
    }

    void calculatePosition()
    {
        linear_vel = (actual_w_left+actual_w_right)/2.0;
        angular_vel = (actual_w_right - actual_w_left)/(base);

        //ROS_INFO("DT right: %f, DT left: %f", dt_right, dt_left);
        //ROS_INFO("Wheel Left Odom: %f, Wheel Right Odom: %f",actual_w_left,actual_w_right);
        //ROS_INFO("ODOM Delta Encoder Right: %f, Delta Encoder Left: %f", delta_enc_right, delta_enc_left);
        ROS_INFO("Linear Velocity: %f \nAngular Velocity: %f", linear_vel,angular_vel);

        msg_time = ros::Time::now();

        dt = (msg_time-last_msg_time).toSec();


        last_msg_time = ros::Time::now();
        delta_phi = angular_vel*dt;
        phi = phi + delta_phi;

        if(phi >= 2* M_PI){
            phi = phi - 2* M_PI;
        }else if(phi < 0){
          phi = phi + 2 * M_PI;
        }else{
          phi = phi;
        }

        delta_x = linear_vel*cos(phi);
        delta_y = linear_vel*sin(phi);

        x = x+delta_x*dt;
        y = y+delta_y*dt;

        //Publish the Odometry over TF

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(phi);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = msg_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        //Publish the Odometry as a message over ROS

        odom_msg.header.stamp = msg_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Position
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        //velocity

        odom_msg.twist.twist.linear.x = linear_vel;
        odom_msg.twist.twist.angular.z = angular_vel;



        pub_odom.publish(odom_msg);
        //ROS_INFO("Position (x,y,phi) = (%f,%f,%f)", x,y,phi);
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
    nav_msgs::Odometry odom_msg;
    ros::Time msg_time;
    ros::Time last_msg_time;
    ros::Time last_msg_time_left;
    ros::Time last_msg_time_right;
    bool first_msg_left;
    bool first_msg_right;

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
    float dt_left;
    float dt_right;
    float dt;

    float linear_vel;
    float angular_vel;

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
