#include <ros/ros.h>
#include <mrover/LED.h>
#include <cstdint>
#include <mrover/CAN.h>

void changeLED(const mrover::LED::ConstPtr& msg);

ros::Publisher CANPublisher;

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "led_hw_bridge");
    ros::NodeHandle nh;

    CANPublisher = nh.advertise<mrover::CAN>("can_requests", 1);
    // Subscribe to the ROS topic for arm commands
    ros::Subscriber moveArmSubscriber = nh.subscribe<mrover::LED>("led_hw_bridge", 1, changeLED);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void changeLED(const mrover::LED::ConstPtr& msg) {
    mrover::CAN CANRequest;
    CANRequest.bus = 0;  // TODO
    CANRequest.id = 0;
    uint8_t data = msg->blue | (msg->red & 0b1) | (msg->green << 1) | (msg->blue << 2) | (msg->is_blinking << 3);
    CANRequest.data.push_back(data);
    CANPublisher.publish(CANRequest);
}