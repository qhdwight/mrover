#pragma once

#include <string>

#include <ros/ros.h>

#include <can_device.hpp>
#include <units/units.hpp>

namespace mrover {

    class Controller {
    public:
        Controller(ros::NodeHandle const& nh, std::string name, std::string controller_name)
            : mNh{nh},
              mName{std::move(name)},
              mControllerName{std::move(controller_name)},
              mDevice{nh, mName, mControllerName},
              mIncomingCANSub{mNh.subscribe<CAN>(std::format("can/{}/in", name), 16, &Controller::processCANMessage, this)} {
        }

        virtual ~Controller() = default;

        // TODO: receive information

        virtual void set_desired_throttle(Percent throttle) = 0;          // from -1.0 to 1.0
        virtual void set_desired_velocity(RadiansPerSecond velocity) = 0; // joint output
        virtual void set_desired_position(Radians position) = 0;          // joint output
        virtual void processCANMessage(CAN::ConstPtr const& msg) = 0;

    protected:
        ros::NodeHandle mNh;
        std::string mName, mControllerName;
        CanDevice mDevice;
        ros::Subscriber mIncomingCANSub;
        RadiansPerSecond mMinVelocity{}, mMaxVelocity{};
        Radians mMinPosition{}, mMaxPosition{};

        //    virtual void send_CAN_frame(uint64_t frame) = 0;
    };

} // namespace mrover
