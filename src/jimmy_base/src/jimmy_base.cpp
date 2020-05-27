/*******************************************************************************
Author: Emiliano Borghi wrote this code: https://github.com/eborghi10/my_ROS_mobile_robot/blob/master/my_robot_base/src/my_robot_base.cpp
        and I (MO) modified it to work for my project. 

Purpose: This code uses the read and write functions I implemented in this 
         package with the purpose of using a diff_drive controller within
         the ros_control library. The control_manager node will load a diff_drive
         controller and this controller will interface with the hardware through
         this code. 

Date: Dec, 2019

*******************************************************************************/
#include <chrono>
#include <functional>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <jimmy_hw_interface.h>

void controlLoop(JimmyHWInterface &hw, controller_manager::ControllerManager &cm, 
                 std::chrono::system_clock::time_point &last_time)
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw.read(elapsed); //here we call the read method to get encoder data of wheels.
    cm.update(ros::Time::now(), elapsed);
    hw.write(); //here we give commands to the motors
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jimmy_base_node");

    JimmyHWInterface hw;
    controller_manager::ControllerManager cm(&hw, hw.nh);
    
    //this is a system to pass around variables to initialze various nodes.
    double control_frequency = 10.0;
    //hw.private_nh.param<double>("control_frequency", control_frequency, 10.0);

    ros::CallbackQueue jimmy_queue;
    ros::AsyncSpinner jimmy_spinner(1, &jimmy_queue);

    std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();
    ros::TimerOptions control_timer(
        ros::Duration(1 / control_frequency), 
        std::bind(controlLoop, std::ref(hw), std::ref(cm), std::ref(last_time)), 
        &jimmy_queue);
    ros::Timer control_loop = hw.nh.createTimer(control_timer);
    jimmy_spinner.start();
    ros::spin();

    return 0;
}
