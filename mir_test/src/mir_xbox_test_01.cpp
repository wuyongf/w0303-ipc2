//
// Created by wuyongf on 14/12/2020.
// Modified date: 27/02/2021
//

#include "../include/mir_test/mir_xbox_test_01.h"

// ros control. joystick
//
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

// time
#include <chrono>
#include <thread>

// sql
#include "../include/w0303-ipc2/sql_ubuntu.h"

class TeleopMir
{
public:
    TeleopMir();
    virtual ~TeleopMir(){};

private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void ThreadKeepMoving(const bool& thread_flag, const bool& keep_moving_flag,
                          const double& latest_linear_velocity, const double& latest_angular_velocity);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_, a_scale_;
    int safety_button_x_, safety_button_rt_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

    double latest_linear_velocity;
    double latest_angular_velocity;

    bool thread_flag = true;
    bool keep_moving_flag = false;

    std::thread th_keep_moving;

    float average_speed;

    // sql
    yf::sql::sql_server sql_ubuntu;

    int sys_control_mode_;
};


TeleopMir::TeleopMir():
        linear_(1),
        angular_(0),
        l_scale_(0.2),
        a_scale_(0.2),
        safety_button_x_(2),
        safety_button_rt_(5)
{

    th_keep_moving = std::thread( &TeleopMir::ThreadKeepMoving, this,
            std::ref(thread_flag), std::ref(keep_moving_flag),
            std::ref(latest_linear_velocity), std::ref(latest_angular_velocity));

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("safety_button_x", safety_button_x_, safety_button_x_);
    nh_.param("safety_button_rt", safety_button_rt_, safety_button_rt_);

    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopMir::joyCallback, this);

}

void TeleopMir::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // 1. Check sys control mode: 0 --- auto; 1 --- manual; 2 --- manual_setting
    sys_control_mode_ = sql_ubuntu.GetSysControlMode();

    if(sys_control_mode_ == 1 || sys_control_mode_ == 2)
    {
        keep_moving_flag = false;
        latest_linear_velocity = l_scale_*joy->axes[linear_];
        latest_angular_velocity = a_scale_*joy->axes[angular_];

        // safety trigger
        //
        if(joy->axes[safety_button_rt_] != -1 || joy->buttons[safety_button_x_] != 1)
        {
            geometry_msgs::TwistStamped twist;

            twist.twist.angular.z = 0;
            twist.twist.linear.x = 0;
            vel_pub_.publish(twist);
        }

        // high speed mode
        //
        if (joy->axes[safety_button_rt_] == -1 && joy->buttons[safety_button_x_] == 1)
        {
            geometry_msgs::TwistStamped twist;

            if (a_scale_*joy->axes[angular_] == 0 && l_scale_*joy->axes[linear_] == 0)
            {
                twist.twist.angular.z = 0;
                twist.twist.linear.x = 0;
                vel_pub_.publish(twist);
            }

            if (a_scale_*joy->axes[angular_] != 0 || l_scale_*joy->axes[linear_] != 0)
            {
                twist.twist.angular.z = latest_angular_velocity;
                twist.twist.linear.x = latest_linear_velocity;
                vel_pub_.publish(twist);
            }

            if ( joy->axes[angular_] > 0.9 || joy->axes[angular_] < -0.9 ||
                 joy->axes[linear_]  > 0.9 || joy->axes[linear_]  < -0.9 )
            {
                keep_moving_flag = true;
            }
        }

        //todo: (1) low speed mode (button_rt & button_y)
        //      (2) fine tune the average_speed?? > 0.9 ===> > 0.7?

    }
    else
    {
        // do nothing, no need to response.
    }
}

void TeleopMir::ThreadKeepMoving(const bool& thread_flag, const bool& keep_moving_flag,
                                 const double& latest_linear_velocity, const double& latest_angular_velocity)
{
    geometry_msgs::TwistStamped twist_max;

    while(thread_flag)
    {
        while (keep_moving_flag)
        {
            twist_max.twist.angular.z = latest_angular_velocity;
            twist_max.twist.linear.x = latest_linear_velocity;;

            vel_pub_.publish(twist_max);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep for 100 ms
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Sleep for 200 ms
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_mir");

    TeleopMir teleop_mir;

    ros::spin();
}



// joystick -- version: v1.0

// problem:
//  (1) move smoothly.
//      a. if there is any tiny change of joystick, the robot will move
//      b. if the joystick is pulled to edge, the robot will move continuously
//    **c. however, the robot will stop while the joystick was hold somewhere not edge.

// todo:
//  1. connection with ipc1 and sql server
//  2. move smoothly function.
