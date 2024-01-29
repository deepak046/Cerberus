#include "ros/ros.h"
#include <vector>
#include "sensor_msgs/JointState.h"
#include <unitree_legged_msgs/HighState.h> // Unitree High State message type
#include <unitree_legged_msgs/LowState.h> // Unitree A1 State message type
#include <unitree_legged_msgs/MotorState.h> // Unitree Motor State message type
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h" 

// Include eigen3 for matrix operations
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/bind.hpp>

// Advertise the topic
ros::Publisher pub_joint;
ros::Publisher pub_imu;


void sensor_callback(const unitree_legged_msgs::LowState::ConstPtr &low_state_msg) {
/*
    Description:- 
        Callback function for sensor data. This function is called when sensor data is received. 
        It processes the received data and publishes the joint states and IMU data.

    Params:- 
        low_state_msg: The received LowState message from the Unitree robot

    Publishes:-
        joint_foot_msg: The joint states of the robot in 'Ceberus VILO' format
        imu_msg: The IMU data of the robot
*/

    sensor_msgs::JointState joint_foot_msg;

    static const int NUM_LEG = 4; // Number of legs
    static const int NUM_DOF = 12; // Number of joints in the robot

    joint_foot_msg.header = low_state_msg->header;

    // The order fo the joints is different to the order in the URDF of pbl_robodog
    joint_foot_msg.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                            "FL_foot", "FR_foot", "RL_foot", "RR_foot"};
    joint_foot_msg.position.resize(NUM_DOF + NUM_LEG);
    joint_foot_msg.velocity.resize(NUM_DOF + NUM_LEG);
    joint_foot_msg.effort.resize(NUM_DOF + NUM_LEG);

    // The different loops are necessary because the order is messed ups
    for (int i=0; i<3; ++i)
    {
        joint_foot_msg.position[i] = low_state_msg->motorState[i+3].q;
        joint_foot_msg.velocity[i] = low_state_msg->motorState[i+3].dq;
        joint_foot_msg.effort[i] = 0;
    }
    for (int i=3; i<6; ++i)
    {
        joint_foot_msg.position[i] = low_state_msg->motorState[i-3].q;
        joint_foot_msg.velocity[i] = low_state_msg->motorState[i-3].dq;
        joint_foot_msg.effort[i] = 0;
    }    
    for (int i=6; i<9; ++i)
    {
        joint_foot_msg.position[i] = low_state_msg->motorState[i+3].q;
        joint_foot_msg.velocity[i] = low_state_msg->motorState[i+3].dq;
        joint_foot_msg.effort[i] = 0;
    }
    for (int i=9; i<12; ++i)
    {
        joint_foot_msg.position[i] = low_state_msg->motorState[i-3].q;
        joint_foot_msg.velocity[i] = low_state_msg->motorState[i-3].dq;
        joint_foot_msg.effort[i] = 0;
    }
    for (int i=NUM_DOF; i<NUM_DOF+NUM_LEG; ++i)
    {
        joint_foot_msg.position[i] = 0;
        joint_foot_msg.velocity[i] = 0;
        joint_foot_msg.effort[i] = 0;
    }

    // FL_foot force
    joint_foot_msg.effort[12] = static_cast<double>(low_state_msg->footForce[1]);
    // FR_foot force
    joint_foot_msg.effort[13] = static_cast<double>(low_state_msg->footForce[0]);
    // RL_foot force
    joint_foot_msg.effort[14] = static_cast<double>(low_state_msg->footForce[3]);
    // RR_foot force
    joint_foot_msg.effort[15] = static_cast<double>(low_state_msg->footForce[2]);

    pub_joint.publish(joint_foot_msg);


    // Publishes IMU data to be in sync with the joint_foot data 
    sensor_msgs::Imu imu_msg;
    imu_msg.header = low_state_msg->header;
    imu_msg.orientation.x = low_state_msg->imu.quaternion[0];
    imu_msg.orientation.y = low_state_msg->imu.quaternion[1];
    imu_msg.orientation.z = low_state_msg->imu.quaternion[2];
    imu_msg.orientation.w = low_state_msg->imu.quaternion[3];
    imu_msg.angular_velocity.x = low_state_msg->imu.gyroscope[0];
    imu_msg.angular_velocity.y = low_state_msg->imu.gyroscope[1];
    imu_msg.angular_velocity.z = low_state_msg->imu.gyroscope[2];
    imu_msg.linear_acceleration.x = low_state_msg->imu.accelerometer[0];
    imu_msg.linear_acceleration.y = low_state_msg->imu.accelerometer[1];
    imu_msg.linear_acceleration.z = low_state_msg->imu.accelerometer[2];

    pub_imu.publish(imu_msg);
}


int main(int argc, char** argv) {
  
    // Initialize the ROS node
    ros::init(argc, argv, "joint_foot_pub");
    ros::NodeHandle nh_("~");

    // Advertise the topics to be published
    pub_joint = nh_.advertise<sensor_msgs::JointState>("/cerberus/joint_foot", 30); 
    pub_imu = nh_.advertise<sensor_msgs::Imu>("/cerberus/imu", 30);

    // Subscribe to the topic with the sensor data
    ros::Subscriber sub = nh_.subscribe("/robodog_rl_controller/low_state", 30, &sensor_callback);

    ros::spin();
    return 0;
}
