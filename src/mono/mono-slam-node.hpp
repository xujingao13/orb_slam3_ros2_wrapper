/**
 * @file mono-slam-node.hpp
 * @brief Definition of the MonoSlamNode Wrapper class.
 * @author Suchetan R S (rssuchetan@gmail.com)
 */

#ifndef MONO_SLAM_NODE_HPP_
#define MONO_SLAM_NODE_HPP_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <slam_msgs/msg/map_data.hpp>
#include <slam_msgs/srv/get_map.hpp>

#include "orb_slam3_ros2_wrapper/type_conversion.hpp"
#include "orb_slam3_ros2_wrapper/orb_slam3_interface.hpp"

namespace ORB_SLAM3_Wrapper
{
    class MonoSlamNode : public rclcpp::Node
    {
    public:
        MonoSlamNode(const std::string &strVocFile,
                     const std::string &strSettingsFile,
                     ORB_SLAM3::System::eSensor sensor);
        ~MonoSlamNode();

    private:
        // ROS 2 Callbacks.
        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU);
        void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msgOdom);
        void MONOCallback(const sensor_msgs::msg::Image::SharedPtr msgRGB);

        /**
         * @brief Publishes map data. (Keyframes and all poses in the current active map.)
         * @param orb_atlas Pointer to the Atlas object.
         * @param last_init_kf_id ID of the last initialized keyframe.
         */
        void publishCurrentMapPointCloud();
        void publishReferenceMapPointCloud();
        void combinedPublishCallback();
        
        /**
         * Member variables
         */
        // MONO Sensor specifics
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgbSub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
        // ROS Publishers and Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
        //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapPointsPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr currentMapPointsPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr referenceMapPointsPub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cameraPosePub_;
        // TF
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tfListener_;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
        
        rclcpp::Service<slam_msgs::srv::GetMap>::SharedPtr getMapDataService_;
        // ROS Timers
        rclcpp::CallbackGroup::SharedPtr mapPointsCallbackGroup_;
        rclcpp::TimerBase::SharedPtr mapPointsTimer_;
        // ROS Params
        std::string robot_base_frame_id_;
        std::string odom_frame_id_;
        std::string global_frame_;
        double robot_x_, robot_y_;
        bool rosViz_;
        bool isTracked_ = false;
        bool no_odometry_mode_;
        double frequency_tracker_count_ = 0;
        int map_data_publish_frequency_;
        int landmark_publish_frequency_;
        std::chrono::_V2::system_clock::time_point frequency_tracker_clock_;

        ORB_SLAM3_Wrapper::WrapperTypeConversions typeConversion_;
        std::shared_ptr<ORB_SLAM3_Wrapper::ORBSLAM3Interface> interface_;
        geometry_msgs::msg::TransformStamped tfMapOdom_;
    };
}
#endif
