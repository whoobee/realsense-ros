#pragma once

#include <base_realsense_node.h>

namespace realsense2_camera
{
    class T265RealsenseNode : public BaseRealSenseNode
    {
        public:
            T265RealsenseNode(ros::NodeHandle& nodeHandle,
                          ros::NodeHandle& privateNodeHandle,
                          rs2::device dev,
                          const std::string& serial_no);
            virtual void toggleSensors(bool enabled) override;
            ~T265RealsenseNode();
            virtual void publishTopics() override;

        protected:
            void calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile) override;

        private:
            void initializeOdometryInput();
            void importLocalization();
            void importLocalization(const std::string& localization_file);
            void exportLocalization();
            void exportLocalization(const std::string& export_file);
            void initMapFrame(bool relocalizing);
            void setupSubscribers();
            void handleWarning();   
            void odom_in_callback(const nav_msgs::Odometry::ConstPtr& msg);
            void warningDiagnostic (diagnostic_updater::DiagnosticStatusWrapper &stat);
            diagnostic_updater::Updater callback_updater;

            rs2::pose_sensor _pose_snr;
            ros::Subscriber _odom_subscriber;
            rs2::wheel_odometer _wo_snr;
            bool _use_odom_in;
            std::string  _T265_fault;
            ros::Timer _timer;
            bool relocalization_pose_initialized;
            tf2_ros::TransformBroadcaster _dynamic_tf_broadcaster;
    };
}
