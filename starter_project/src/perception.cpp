#include "perception.hpp"

// ROS Headers, ros namespace
#include <image_transport/image_transport.h>
#include <ros/init.h>

#include <numeric>

int main(int argc, char** argv) {
    ros::init(argc, argv, "starter_project_perception"); // Our node name (See: http://wiki.ros.org/Nodes)

    [[maybe_unused]] mrover::Perception perception;

    // "spin" blocks until our node dies
    // It listens for new messages and calls our subscribed functions with them
    ros::spin();

    return EXIT_SUCCESS;
}

namespace mrover {

    Perception::Perception() : mNodeHandle{} {
        image_transport::ImageTransport imageTransport(mNodeHandle);
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        imageTransport.subscribe("camera/color/image_raw", 1, &Perception::imageCallback, this);

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);
    }

    void Perception::imageCallback(sensor_msgs::ImageConstPtr const& image) {
        cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        // Detect tags in the image pixels
        findTagsInImage(cvImage, mTags);
        // Select the tag that is closest to the middle of the screen
        StarterProjectTag tag = selectTag(mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv_bridge::CvImagePtr const& image, std::vector<StarterProjectTag>& tags) {
        tags.clear(); // Clear old tags in output vector

        cv::aruco::detectMarkers(image->image, mTagDictionary, mTagCorners, mTagIds);

        // Always use "size_t" to iterate containers
        // Would have used zip from <ranges> but we don't have C++20 on ROS 1 :(
        for (size_t i = 0; i < mTagIds.size(); ++i) {
            StarterProjectTag tag{};
            tag.tagId = mTagIds[i];
            // std::tie allows us to assign x and y from a std::pair
            std::tie(tag.xTagCenterPixel, tag.yTagCenterPixel) = getCenterFromTagCorners(mTagCorners[i]);
            tag.closenessMetric = getClosenessMetricFromTagCorners(image->image, mTagCorners[i]);
            tags.push_back(tag);
        }
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) {
        auto it = std::min_element(tags.begin(), tags.end(), [](StarterProjectTag const& t1, StarterProjectTag const& t2) {
            return t1.xTagCenterPixel < t2.xTagCenterPixel;
        });
        return *it;
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) {
        // idea: the closer we are to the target the bigger area our tag will take up in image space
        auto imageArea = static_cast<float>(image.size().area());
        auto tagArea = static_cast<float>(cv::contourArea(tagCorners));
        // ratio will always be in the range [0, 1]
        return tagArea / imageArea;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) {
        cv::Point2f center = std::accumulate(tagCorners.begin(), tagCorners.end(), cv::Point2f{}) / static_cast<float>(tagCorners.size());
        return {center.x, center.y};
    }

} // namespace mrover
