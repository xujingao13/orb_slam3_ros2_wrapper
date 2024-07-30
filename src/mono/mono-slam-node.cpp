/**
 * @file mono-slam-node.cpp
 * @brief Implementation of the MonoSlamNode Wrapper class.
 * @author Suchetan R S (rssuchetan@gmail.com)
 */
#include "mono-slam-node.hpp"

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3_Wrapper
{
    MonoSlamNode::MonoSlamNode(const std::string &strVocFile,
                               const std::string &strSettingsFile,
                               ORB_SLAM3::System::eSensor sensor)
        : Node("ORB_SLAM3_MONO_ROS2")
    {
        // ROS Subscribers
        rgbSub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw",10,std::bind(&MonoSlamNode::MONOCallback,
        									    	    this,std::placeholders::_1));

        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1000, std::bind(&MonoSlamNode::ImuCallback, this, std::placeholders::_1));
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1000, std::bind(&MonoSlamNode::OdomCallback, this, std::placeholders::_1));
        // ROS Publishers 
        // mapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mono_map_points", 10);
        currentMapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mono_current_map_points", 10);
        referenceMapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mono_reference_map_points", 10);
        cameraPosePub_ = this->create_publisher<geometry_msgs::msg::Pose>("camera_pose", 10);
        
        // TF
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        bool bUseViewer;
        this->declare_parameter("visualization", rclcpp::ParameterValue(true));
        this->get_parameter("visualization", bUseViewer);

        this->declare_parameter("ros_visualization", rclcpp::ParameterValue(false));
        this->get_parameter("ros_visualization", rosViz_);

        this->declare_parameter("robot_base_frame", "base_link");
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("global_frame", "map");
        this->get_parameter("global_frame", global_frame_);

        this->declare_parameter("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_id_);

        this->declare_parameter("robot_x", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_x", robot_x_);

        this->declare_parameter("robot_y", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_y", robot_y_);

        this->declare_parameter("no_odometry_mode", rclcpp::ParameterValue(false));
        this->get_parameter("no_odometry_mode", no_odometry_mode_);

        this->declare_parameter("map_data_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("map_data_publish_frequency", map_data_publish_frequency_);

        this->declare_parameter("landmark_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("landmark_publish_frequency", landmark_publish_frequency_);
        
	    mapPointsCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // mapPointsTimer_ = this->create_wall_timer(std::chrono::milliseconds(landmark_publish_frequency_), std::bind(&MonoSlamNode::publishCurrentMapPointCloud, this));
        mapPointsTimer_ = this->create_wall_timer(std::chrono::milliseconds(landmark_publish_frequency_), std::bind(&MonoSlamNode::combinedPublishCallback, this));


        interface_ = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(strVocFile, strSettingsFile,
                                                                            sensor, bUseViewer, rosViz_, robot_x_,
                                                                            robot_y_, global_frame_, odom_frame_id_, robot_base_frame_id_);
        frequency_tracker_count_ = 0;
        frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR END!");
    }

    MonoSlamNode::~MonoSlamNode()
    {
        rgbSub_.reset();
        imuSub_.reset();
        odomSub_.reset();
        interface_.reset();
        RCLCPP_INFO(this->get_logger(), "DESTRUCTOR!");
    }

    void MonoSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "ImuCallback");
        // push value to imu buffer.
        interface_->handleIMU(msgIMU);
    }

    void MonoSlamNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
    {
        if(!no_odometry_mode_)
        {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "OdomCallback");
            interface_->getMapToOdomTF(msgOdom, tfMapOdom_);
        }
        else RCLCPP_WARN(this->get_logger(), "Odometry msg recorded but no odometry mode is true, set to false to use this odometry");
    }
    //复刻
    void MonoSlamNode::MONOCallback(const sensor_msgs::msg::Image::SharedPtr msgRGB)
    {
        Sophus::SE3f Tcw;
        if (interface_->trackMONO(msgRGB, Tcw))
        {
            isTracked_ = true;
            if(no_odometry_mode_) interface_->getDirectMapToRobotTF(msgRGB->header, tfMapOdom_);
            tfBroadcaster_->sendTransform(tfMapOdom_);
            ++frequency_tracker_count_;

            // publish camera's pose
            auto camPose = typeConversion_.se3ToPoseMsg(Tcw);
            cameraPosePub_->publish(camPose);

            // publishMapPointCloud();
            // std::thread(&MonoSlamNode::publishMapPointCloud, this).detach();
        }
    }
    //复刻
    void MonoSlamNode::publishCurrentMapPointCloud()
    {
        if (isTracked_)
        {
            // Using high resolution clock to measure time
            auto start = std::chrono::high_resolution_clock::now();

            sensor_msgs::msg::PointCloud2 mapPCL;

            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_create_mapPCL = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to create mapPCL object: " << time_create_mapPCL << " seconds");

            interface_->getCurrentMapPoints(mapPCL);
            // interface_->getReferenceMapPoints(mapPCL);

            if(mapPCL.data.size() == 0)
                return;

            auto t2 = std::chrono::high_resolution_clock::now();
            auto time_get_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to get current map points: " << time_get_map_points << " seconds");

            currentMapPointsPub_->publish(mapPCL);
            auto t3 = std::chrono::high_resolution_clock::now();
            auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to publish map points: " << time_publish_map_points << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "=======================");


            // Calculate the time taken for each line

            // Print the time taken for each line
        }
    }

    void MonoSlamNode::publishReferenceMapPointCloud()
    {
        if (isTracked_)
        {
            // Using high resolution clock to measure time
            auto start = std::chrono::high_resolution_clock::now();

            sensor_msgs::msg::PointCloud2 mapPCL;

            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_create_mapPCL = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to create mapPCL object: " << time_create_mapPCL << " seconds");

            // interface_->getCurrentMapPoints(mapPCL);
            interface_->getReferenceMapPoints(mapPCL);

            if(mapPCL.data.size() == 0)
                return;

            auto t2 = std::chrono::high_resolution_clock::now();
            auto time_get_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to get current map points: " << time_get_map_points << " seconds");

            referenceMapPointsPub_->publish(mapPCL);
            auto t3 = std::chrono::high_resolution_clock::now();
            auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to publish map points: " << time_publish_map_points << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "=======================");


            // Calculate the time taken for each line

            // Print the time taken for each line
        }
    }

    void MonoSlamNode::combinedPublishCallback() 
    {
        this->publishCurrentMapPointCloud();
        this->publishReferenceMapPointCloud();
    }


}
