#include "../include/t265_realsense_node.h"

using namespace realsense2_camera;

T265RealsenseNode::T265RealsenseNode(ros::NodeHandle& nodeHandle,
                                     ros::NodeHandle& privateNodeHandle,
                                     rs2::device dev,
                                     const std::string& serial_no) : 
                                     BaseRealSenseNode(nodeHandle, privateNodeHandle, dev, serial_no),
                                     _pose_snr(dev.first<rs2::pose_sensor>()),
                                     _wo_snr(dev.first<rs2::wheel_odometer>()),
                                     _use_odom_in(false) 
                                     {
                                         _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_MOTION_MODULE_TEMPERATURE};
                                         initializeOdometryInput();
                                         handleWarning();
                                         importLocalization();
                                     }

T265RealsenseNode::~T265RealsenseNode()
{
    exportLocalization();
}

void T265RealsenseNode::importLocalization()
{
    std::string localization_file;
    _pnh.param("map_in", localization_file, std::string(""));
    if (localization_file.empty())
    {
        ROS_INFO("No [map_in] specified. No localization data loaded.");

        initMapFrame(false);
        return;
    }

    importLocalization(localization_file);
    initMapFrame(true);
}

void T265RealsenseNode::exportLocalization()
{
    std::string localization_file;
    _pnh.param("map_out", localization_file, std::string(""));
    if (localization_file.empty())
    {
        ROS_INFO("No [map_out]. No localization data loaded.");
        return;
    }

    exportLocalization(localization_file);
}

void rsPoseToMsg(const rs2_pose& pose, geometry_msgs::Pose& pose_msg)
{
    pose_msg.position.x = -pose.translation.z;
    pose_msg.position.y = -pose.translation.x;
    pose_msg.position.z = pose.translation.y;
    pose_msg.orientation.x = -pose.rotation.z;
    pose_msg.orientation.y = -pose.rotation.x;
    pose_msg.orientation.z = pose.rotation.y;
    pose_msg.orientation.w = pose.rotation.w;
}

// Copied from https://github.com/IntelRealSense/librealsense/blob/master/examples/ar-advanced/rs-ar-advanced.cpp
std::vector<uint8_t> bytes_from_raw_file(const std::string& filename)
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

void raw_file_from_bytes(const std::string& filename, const std::vector<uint8_t> bytes)
{
  std::ofstream file(filename, std::ios::binary | std::ios::trunc);
  if (!file.good())
    throw std::runtime_error("Invalid binary file specified. Verify the target path and location permissions");
  file.write((char*)bytes.data(), bytes.size());
}

void T265RealsenseNode::importLocalization(const std::string& localization_file)
{
    try {
      _pose_snr.import_localization_map(bytes_from_raw_file(localization_file));
      ROS_INFO_STREAM("Map loaded from " << localization_file);
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error loading map from " << localization_file << ": " << e.what());
    }

    /**
     * Load relocalization map onto device. Only one relocalization map can be imported at a time;
     * any previously existing map will be overwritten.
     * The imported map exists simultaneously with the map created during the most recent tracking session after start(),
     * and they are merged after the imported map is relocalized.
     * This operation must be done before start().
     * \param[in] lmap_buf map data as a binary blob
     * \return true if success
     */
    //bool import_localization_map(const std::vector<uint8_t>& lmap_buf) const

    /**
     * Get relocalization map that is currently on device, created and updated during most recent tracking session.
     * Can be called before or after stop().
     * \return map data as a binary blob
     */
    //std::vector<uint8_t> export_localization_map() const


    /**
     * Creates a named virtual landmark in the current map, known as static node.
     * The static node's pose is provided relative to the origin of current coordinate system of device poses.
     * This function fails if the current tracker confidence is below 3 (high confidence).
     * \param[in] guid unique name of the static node (limited to 127 chars).
     * If a static node with the same name already exists in the current map or the imported map, the static node is overwritten.
     * \param[in] pos position of the static node in the 3D space.
     * \param[in] orient_quat orientation of the static node in the 3D space, represented by a unit quaternion.
     * \return true if success.
     */
    //set_static_node(const std::string& guid, const rs2_vector& pos, const rs2_quaternion& orient) const

    /**
     * Gets the current pose of a static node that was created in the current map or in an imported map.
     * Static nodes of imported maps are available after relocalizing the imported map.
     * The static node's pose is returned relative to the current origin of coordinates of device poses.
     * Thus, poses of static nodes of an imported map are consistent with current device poses after relocalization.
     * This function fails if the current tracker confidence is below 3 (high confidence).
     * \param[in] guid unique name of the static node (limited to 127 chars).
     * \param[out] pos position of the static node in the 3D space.
     * \param[out] orient_quat orientation of the static node in the 3D space, represented by a unit quaternion.
     * \return true if success.
     */
    //bool get_static_node(const std::string& guid, rs2_vector& pos, rs2_quaternion& orient) const
    //bool remove_static_node(const std::string& guid) const

}

void T265RealsenseNode::initMapFrame(bool relocalizing)
{
    relocalization_pose_initialized=false;

    std::string mapping_guid;
    _pnh.param("mapping_guid", mapping_guid, std::string("map"));
    if (mapping_guid.empty())
    {
        ROS_INFO("No [mapping_guid] specified. This should disable some functionality...");
        return;
    }

    std::string map_frame_id;
    _pnh.param("map_frame_id", map_frame_id, std::string(""));
    if (map_frame_id.empty())
    {
        ROS_INFO("No [map_frame_id] specified. Transform will not be published.");
        return;
    }
    
    
    auto send_transform = [&](const ros::Time& t,
        const float3& trans,
        const tf::Quaternion& q,
        const std::string& from,
        const std::string& to)
    {
        std::vector<geometry_msgs::TransformStamped> tf_msgs;

        geometry_msgs::TransformStamped msg;
        msg.header.stamp = t;
        msg.header.frame_id = from;
        msg.child_frame_id = to;
        msg.transform.translation.x = trans.x;
        msg.transform.translation.y = trans.y;
        msg.transform.translation.z = trans.z;
        msg.transform.rotation.x = q.getX();
        msg.transform.rotation.y = q.getY();
        msg.transform.rotation.z = q.getZ();
        msg.transform.rotation.w = q.getW();
        tf_msgs.push_back(msg);

        _dynamic_tf_broadcaster.sendTransform(tf_msgs);
    };

    auto attempt_localization = [this, mapping_guid, map_frame_id, send_transform]()
    {
        // Get static node if available
        rs2_pose object_pose_in_world;
        if (_pose_snr.get_static_node(mapping_guid, object_pose_in_world.translation, object_pose_in_world.rotation)) {
            std::cout << "Reference frame localized:  " << object_pose_in_world.translation << std::endl;
            relocalization_pose_initialized = true;

            float3 translation;
            translation.x = -object_pose_in_world.translation.z;
            translation.y = -object_pose_in_world.translation.x;
            translation.z = object_pose_in_world.translation.y;

            tf::Quaternion rotation(
                -object_pose_in_world.rotation.z,
                -object_pose_in_world.rotation.x,
                object_pose_in_world.rotation.y,
                object_pose_in_world.rotation.w);

            send_transform(ros::Time::now(), translation, rotation, _odom_frame_id, map_frame_id);
        }
        else
        {
            ROS_WARN_STREAM("Warning, unable to find static node for guid [" << mapping_guid << "]");
        }

        };

    boost::function<void (const ros::TimerEvent&)> getter_callback = [this, attempt_localization, mapping_guid, map_frame_id] (const ros::TimerEvent& event)
    {
        attempt_localization();
    };

    _pose_snr.set_notifications_callback([&](const rs2::notification& n) {
        if (n.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
            ROS_INFO("Relocalization Event Detected.");
            attempt_localization();
        }
    });


    if (relocalizing)
    {
        ROS_INFO("Attempting to relocalize...");

        _timer = _node_handle.createTimer(ros::Duration(1), getter_callback);
    }
    else
    {
        ROS_INFO("Waiting for first pose to be available...");
        boost::function<void (const ros::TimerEvent&)> setter_callback = [this, getter_callback, mapping_guid] (const ros::TimerEvent& event)
        {
            // Set static node if possible (pose confidence must be high enough)
            rs2_vector translation;
            translation.x=0;
            translation.y=0;
            translation.z=0;
            rs2_quaternion rotation;
            rotation.x=0;
            rotation.y=0;
            rotation.z=0;
            rotation.w=1;
            if (_pose_snr.set_static_node(mapping_guid, translation, rotation)) {
                ROS_INFO_STREAM("Reference frame initialized: " << translation);
                _timer = _node_handle.createTimer(ros::Duration(1), getter_callback);
            }
            else
            {
                ROS_WARN_STREAM("Warning, unable to set static node for guid [" << mapping_guid << "]");
            }
        };

        _timer = _node_handle.createTimer(ros::Duration(1), setter_callback);

    }
}

//void T265RealsenseNode::set

void T265RealsenseNode::exportLocalization(const std::string& localization_file)
{

    try
    {
        raw_file_from_bytes(localization_file, _pose_snr.export_localization_map());
        ROS_INFO_STREAM("Saved map to " << localization_file);
    }
    catch (std::runtime_error e)
    {
        ROS_ERROR_STREAM("Error loading map from " << localization_file << ": " << e.what());; 
    }
}

void T265RealsenseNode::initializeOdometryInput()
{
    std::string calib_odom_file;
    _pnh.param("calib_odom_file", calib_odom_file, std::string(""));
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
    _pnh.param("topic_odom_in", topic_odom_in, DEFAULT_TOPIC_ODOM_IN);
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
