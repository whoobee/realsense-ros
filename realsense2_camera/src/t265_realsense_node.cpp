#include "../include/t265_realsense_node.h"

using namespace realsense2_camera;

T265RealsenseNode::T265RealsenseNode(ros::NodeHandle& nodeHandle,
                                     ros::NodeHandle& privateNodeHandle,
                                     rs2::device dev,
                                     const std::string& serial_no) : 
                                     BaseRealSenseNode(nodeHandle, privateNodeHandle, dev, serial_no),
                                     _t265_pnh(nodeHandle),
                                     _pose_sensor(dev.first<rs2::pose_sensor>()),
                                     _wo_snr(dev.first<rs2::wheel_odometer>()),
                                     _use_odom_in(false) 
                                     {
                                         _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_MOTION_MODULE_TEMPERATURE};
                                         initializeOdometryInput();
                                         setupMapReutilization();
                                         handleWarning();
                                     }

void T265RealsenseNode::initializeOdometryInput()
{
    std::string calib_odom_file;
    _t265_pnh.param("calib_odom_file", calib_odom_file, std::string(""));
    if (calib_odom_file.empty())
    {
        ROS_INFO("No calib_odom_file. No input odometry accepted.");
        return;
    }
    std::ifstream calibrationFile(calib_odom_file);
    if (!calibrationFile)
    {
        ROS_FATAL_STREAM("calibration_odometry file not found. calib_odom_file = " << calib_odom_file);
        throw std::runtime_error("calibration_odometry file not found" );
    }
    const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
        std::istreambuf_iterator<char>());
    const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

    if (!_wo_snr.load_wheel_odometery_config(wo_calib))
    {
        ROS_FATAL_STREAM("Format error in calibration_odometry file: " << calib_odom_file);
        throw std::runtime_error("Format error in calibration_odometry file" );
    }
    _use_odom_in = true;
}

void T265RealsenseNode::setupMapReutilization() {
    // Load map if configured in the launch file
    std::string localization_map_filepath;
    _t265_pnh.param("localization_map_filepath", localization_map_filepath, DEFAULT_LOCALIZATION_MAP_PATH);
    if (localization_map_filepath.empty())
    {
        ROS_INFO("No [localization_map_filepath] specified. No localization data loaded.");
    }
    else
    {
        this->importLocalizationMap(localization_map_filepath);
    }
    // Setup service to save map at runtime
    std::string service_save_map;
    _t265_pnh.param("service_save_map", service_save_map, DEFAULT_SERVICE_SAVE_MAP);
    _save_map_srv = _node_handle.advertiseService(service_save_map, &T265RealsenseNode::saveRelocalizationMapSrv, this);

    // Setup service to save static node at runtime
    std::string service_save_static_node;
    _t265_pnh.param("service_save_static_node", service_save_static_node, DEFAULT_SERVICE_SAVE_STATIC_NODE);
    _save_static_node_srv = _node_handle.advertiseService(service_save_static_node, &T265RealsenseNode::saveStaticNodeSrv, this);
    
    // Setup service to read static node at runtime
    std::string service_read_static_node;
    _t265_pnh.param("service_read_static_node", service_read_static_node, DEFAULT_SERVICE_READ_STATIC_NODE);
    _read_static_node_srv = _node_handle.advertiseService(service_read_static_node, &T265RealsenseNode::readStaticNodeSrv, this);
}

void T265RealsenseNode::toggleSensors(bool enabled)
{
  ROS_WARN_STREAM("toggleSensors method not implemented for T265");
}

void T265RealsenseNode::publishTopics()
{
    BaseRealSenseNode::publishTopics();
    setupSubscribers();
}

void  T265RealsenseNode::handleWarning()
{
    rs2::log_to_callback( rs2_log_severity::RS2_LOG_SEVERITY_WARN, [&]
      ( rs2_log_severity severity, rs2::log_message const & msg ) noexcept {
        _T265_fault =  msg.raw();
        std::array<std::string, 2> list_of_fault{"SLAM_ERROR", "Stream transfer failed, exiting"};
        auto it = std::find_if(begin(list_of_fault), end(list_of_fault),
                  [&](const std::string& s) {return _T265_fault.find(s) != std::string::npos; });
        if (it != end(list_of_fault))
        {
          callback_updater.add("Warning ",this, & T265RealsenseNode::warningDiagnostic);
          callback_updater.force_update();
        }
    });
}

void T265RealsenseNode::setupSubscribers()
{
    if (!_use_odom_in) return;

    std::string topic_odom_in;
    _t265_pnh.param("topic_odom_in", topic_odom_in, DEFAULT_TOPIC_ODOM_IN);
    ROS_INFO_STREAM("Subscribing to in_odom topic: " << topic_odom_in);

    _odom_subscriber = _node_handle.subscribe(topic_odom_in, 1, &T265RealsenseNode::odom_in_callback, this);
}

void T265RealsenseNode::odom_in_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_DEBUG("Got in_odom message");
    rs2_vector velocity {-(float)(msg->twist.twist.linear.y),
                          (float)(msg->twist.twist.linear.z),
                         -(float)(msg->twist.twist.linear.x)};

    ROS_DEBUG_STREAM("Add odom: " << velocity.x << ", " << velocity.y << ", " << velocity.z);
    _wo_snr.send_wheel_odometry(0, 0, velocity);
}

void T265RealsenseNode::calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile)
{
    // Transform base to stream
    tf::Quaternion quaternion_optical;
    quaternion_optical.setRPY(M_PI / 2, 0.0, -M_PI / 2);    //Pose To ROS
    float3 zero_trans{0, 0, 0};

    ros::Time transform_ts_ = ros::Time::now();

    rs2_extrinsics ex;
    try
    {
        ex = getAProfile(stream).get_extrinsics_to(base_profile);
    }
    catch (std::exception& e)
    {
        if (!strcmp(e.what(), "Requested extrinsics are not available!"))
        {
            ROS_WARN_STREAM(e.what() << " : using unity as default.");
            ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
        }
        else
        {
            throw e;
        }
    }
    auto Q = rotationMatrixToQuaternion(ex.rotation);

    _node_position = {ex.translation[0], ex.translation[1], ex.translation[2]};
    _node_orientation = {(float)Q.x(), (float)Q.y(), (float)Q.z(), (float)Q.w()};

    Q = quaternion_optical * Q * quaternion_optical.inverse();
    float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
    if (stream == POSE)
    {
        Q = Q.inverse();
        publish_static_tf(transform_ts_, trans, Q, _frame_id[stream], _base_frame_id);
    }
    else
    {
        publish_static_tf(transform_ts_, trans, Q, _base_frame_id, _frame_id[stream]);
        publish_static_tf(transform_ts_, zero_trans, quaternion_optical, _frame_id[stream], _optical_frame_id[stream]);

        // Add align_depth_to if exist:
        if (_align_depth && _depth_aligned_frame_id.find(stream) != _depth_aligned_frame_id.end())
        {
            publish_static_tf(transform_ts_, trans, Q, _base_frame_id, _depth_aligned_frame_id[stream]);
            publish_static_tf(transform_ts_, zero_trans, quaternion_optical, _depth_aligned_frame_id[stream], _optical_frame_id[stream]);
        }
    }
}

void T265RealsenseNode::warningDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
{
  status.summary(diagnostic_msgs::DiagnosticStatus::WARN, _T265_fault);
}

bool T265RealsenseNode::importLocalizationMap(const std::string &localization_file)
{
    try
    {
      _pose_sensor.import_localization_map(this->bytesFromRawFile(localization_file));
      ROS_INFO_STREAM("T265 Localization map loaded from " << localization_file);
    }
    catch (std::runtime_error& e)
    {
      ROS_WARN_STREAM("Error loading T265 map from " << localization_file << ": " << e.what());
      return false;
    }
    return true;
}

bool T265RealsenseNode::exportLocalizationMap(const std::string &localization_file)
{
    try
    {
        this->rawFileFromBytes(localization_file, _pose_sensor.export_localization_map());
        ROS_INFO_STREAM("Saved T265 map to " << localization_file);
    }
    catch (std::runtime_error& e)
    {
        ROS_WARN_STREAM("Error saving T265 map to " << localization_file << ": " << e.what());
        return false;
    }
    return true;
}

bool T265RealsenseNode::setStaticNode(const std::string &node_name)
{
    try
    {
        if(_pose_sensor.set_static_node(node_name, _node_position, _node_orientation))
        {
            ROS_INFO_STREAM("Static node '" << node_name << "' was sucessfuly set.");
        }
        else
        {
            ROS_INFO_STREAM("Error setting static node: " << node_name << ". Probably the sensor confidence was not high enough.");
            ROS_INFO_STREAM("Try moving the sensor to ensure high confidence level.");

            return false;
        }
    }
    catch (std::runtime_error& e)
    {
        ROS_INFO_STREAM("Exception while setting static node: " << node_name << ".");

        return false;
    }

    return true;
}

geometry_msgs::PoseStamped T265RealsenseNode::getStaticNode(const std::string &node_name, const std::string &node_frame)
{
    geometry_msgs::PoseStamped return_val;

    rs2_vector node_position;
    rs2_quaternion node_orientation;
    
    return_val.header.frame_id=node_frame;
    return_val.header.stamp = ros::Time::now();

    return_val.pose.position.x = 0;
    return_val.pose.position.y = 0;
    return_val.pose.position.z = 0;

    return_val.pose.orientation.x = 0;
    return_val.pose.orientation.y = 0;
    return_val.pose.orientation.z = 0;
    return_val.pose.orientation.w = 0;

    try
    {
        if(_pose_sensor.get_static_node(node_name, node_position, node_orientation))
        {
            tf::Quaternion Q(node_orientation.x, node_orientation.y, node_orientation.z, node_orientation.w);
            tf::Quaternion quaternion_optical;
            quaternion_optical.setRPY(M_PI / 2, 0.0, -M_PI / 2);    //Pose To ROS

            Q = quaternion_optical * Q * quaternion_optical.inverse();
            Q = Q.inverse();

            ROS_INFO_STREAM("Static node '" << node_name << "' was sucessfuly received from the sensor.");
            return_val.pose.position.x = node_position.x;
            return_val.pose.position.y = node_position.y;
            return_val.pose.position.z = node_position.z;

            return_val.pose.orientation.x = Q.x();
            return_val.pose.orientation.y = Q.y();
            return_val.pose.orientation.z = Q.z();
            return_val.pose.orientation.w = Q.w();
        }
        else
        {
            ROS_INFO_STREAM("Error receiving static node: " << node_name << ".");
        }
    }
    catch (std::runtime_error& e)
    {
        ROS_INFO_STREAM("Exception while receiving static node: " << node_name << ".");
    }
    return return_val;
}

bool T265RealsenseNode::saveRelocalizationMapSrv(realsense2_camera::MapPathString::Request &req,
                                                 realsense2_camera::MapPathString::Response &res) {
    res.success = this->exportLocalizationMap(req.filepath);
    return true;
}

bool T265RealsenseNode::saveStaticNodeSrv(realsense2_camera::NodeNameString::Request &req,
                                          realsense2_camera::NodeNameString::Response &res) {
    res.success = this->setStaticNode(req.name);
    return true;
}

bool T265RealsenseNode::readStaticNodeSrv(realsense2_camera::NodePosition::Request &req,
                                          realsense2_camera::NodePosition::Response &res) {
    res.position = this->getStaticNode(req.name, req.frame);
    return true;
}

std::vector<uint8_t> T265RealsenseNode::bytesFromRawFile(const std::string &filename)
{
    std::ifstream file(filename.c_str(), std::ios::binary);
    if (!file.good())
        throw std::runtime_error("Invalid binary file specified. Verify the source path and location permissions");

    // Determine the file length
    file.seekg(0, std::ios_base::end);
    std::size_t size = file.tellg();
    if (!size)
        throw std::runtime_error("Invalid binary file -zero-size");
    file.seekg(0, std::ios_base::beg);

    // Create a vector to store the data
    std::vector<uint8_t> v(size);

    // Load the data
    file.read((char*)&v[0], size);

    return v;
}

void T265RealsenseNode::rawFileFromBytes(const std::string &filename, const std::vector<uint8_t> &bytes)
{
    std::ofstream file(filename, std::ios::binary | std::ios::trunc);
    if (!file.good())
        throw std::runtime_error("Invalid binary file specified. Verify the target path and location permissions");
    file.write((char*)bytes.data(), bytes.size());
}
