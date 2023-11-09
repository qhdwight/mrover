#pragma once

#include <can_device.hpp>
#include <controller.hpp>

#include "messaging.hpp"

namespace mrover {

    class BrushedController : public Controller {
    public:
        void setDesiredThrottle(Percent throttle) override; // from -1.0 to 1.0
        void setDesiredVelocity(RadiansPerSecond velocity) override;
        void setDesiredPosition(Radians position) override;

        void processCANMessage(CAN::ConstPtr const& msg) override;

        void processMessage(ControllerDataState const& state);

        void sendConfiguration();

        double getEffort() override;

        BrushedController(ros::NodeHandle const& nh, std::string name, std::string controllerName);
        ~BrushedController() override = default;

    private:
        bool mIsConfigured = false;
        ConfigCommand mConfigCommand;
        static const std::unordered_map<int, std::string> mCodeToError;
    };

} // namespace mrover
