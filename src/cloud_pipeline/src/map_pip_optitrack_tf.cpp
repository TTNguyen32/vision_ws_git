// Map pipeline for a drone-mounted LiDAR, posed entirely through TF.
// Derived from map_pip_optitrack.cpp (22 Sep 2026): same service-driven
// collection, run directories, live snapshots/meshes, normals, paths.
//
// Pose source:
//   T_world_lidar = tf_buffer_->lookupTransform(frame_id, <cloud frame>,
//                                               <scan stamp>)
// TF composes the dynamic world -> base (mocap bridge) with the static
// base -> lidar extrinsic (static_transform_publisher, in the launcher),
// and interpolates world -> base to the scan stamp itself. No mocap
// subscription and no hand-written lerp/slerp. Scans with no usable
// transform at their stamp are dropped.
//
// Author: Thanh Tin Nguyen
// Email: ttn32@cam.ac.uk

#include <memory>
#include <string>
#include <mutex>
#include <filesystem>
#include <chrono>
#include <limits>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <unordered_map>
#include <thread>
#include <cstdint>
#include <vector>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cstring>                  

#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <nav_msgs/msg/path.hpp>

#include <std_msgs/msg/string.hpp>          // cluster status line for the GUI

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>          // loadPolygonFileSTL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>  // SACSegmentationFromNormals for cylinder fit
#include <pcl/segmentation/extract_clusters.h>  // EuclideanClusterExtraction for cluster removal

#include <Eigen/Geometry>

#include <tf2/exceptions.h>                 // tf2::TransformException
#include <tf2/time.h>                       // tf2::durationFromSec, TimePointZero
#include <tf2_eigen/tf2_eigen.hpp>          // tf2::transformToEigen
#include <tf2_ros/buffer.h>                 // tf2_ros::Buffer (the TF cache)
#include <tf2_ros/transform_listener.h>     // fills the Buffer from /tf, /tf_static

// Simple explicit start/stop timer for testing
class Timer {
public:
  void start() {
    start_ = std::chrono::steady_clock::now();
  }

  double stop_ms() {
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }

private:
  std::chrono::steady_clock::time_point start_;
};

using PointT = pcl::PointXYZ; // create a convenient alias of PointXYZ
Timer t; // declare a timer 

// Find the first field matching any of `names`, or nullptr.
static const sensor_msgs::msg::PointField *
findField(const sensor_msgs::msg::PointCloud2 &msg,
          const std::vector<std::string> &names)
{
    for (const auto &n : names)
        for (const auto &f : msg.fields)
            if (f.name == n) return &f;
    return nullptr;
}

// Read point i's value of field f as a double, whatever its storage type.
// memcpy, not a cast: Livox puts `timestamp` at offset 18, which is not
// 8-byte aligned, so reinterpreting the pointer is undefined behaviour.
static double readFieldAsDouble(const sensor_msgs::msg::PointCloud2 &msg,
                                const sensor_msgs::msg::PointField &f,
                                std::size_t i)
{
    const uint8_t *p = msg.data.data() + i * msg.point_step + f.offset;
    switch (f.datatype)
    {
        case sensor_msgs::msg::PointField::FLOAT64:
        { double v;   std::memcpy(&v, p, 8); return v; }
        case sensor_msgs::msg::PointField::FLOAT32:
        { float v;    std::memcpy(&v, p, 4); return static_cast<double>(v); }
        case sensor_msgs::msg::PointField::UINT32:
        { uint32_t v; std::memcpy(&v, p, 4); return static_cast<double>(v); }
        default:
            return std::numeric_limits<double>::quiet_NaN();
    }
}

struct SelectedTarget
{
    pcl::PointXYZ position;
    pcl::Normal normal;
};

class MapPipeline : public rclcpp::Node
{
public:
    MapPipeline() : Node("map_pip_tf")
    {
        // TOPICS
        //cloud_topic_       = declare_parameter<std::string>("cloud_topic", "/dlio/odom_node/pointcloud/deskewed");
        cloud_topic_       = declare_parameter<std::string>("cloud_topic", "/livox/lidar");
        clicked_topic_     = declare_parameter<std::string>("clicked_topic", "/clicked_point");
        global_map_topic_  = declare_parameter<std::string>("global_map_topic", "/global_map");
        processed_topic_   = declare_parameter<std::string>("processed_topic", "/processed/map");
        //target_topic_      = declare_parameter<std::string>("locked_target", "/processed/locked_target");
        targets_topic_     = declare_parameter<std::string>("targets_topic", "/processed/selected_normals");
        targets_markers_topic_ = declare_parameter<std::string>("targets_markers_topic", "selected_normals_markers");
        path_topic_        = declare_parameter<std::string>("path_topic", "/processed/path");
        rim_topic_         = declare_parameter<std::string>("cylinder_rim", "/processed/cylinder_rim");
        cylinder_marker_topic_= declare_parameter<std::string>("cylinder_axis", "/processed/cylinder_axis");
        mesh_topic_        = declare_parameter<std::string>("mesh_topic", "/processed/mesh");

        // Frames
        // Map is built in frame_id, the root of the TF tree:
        //   frame_id --(dynamic, mocap bridge)--> body_frame
        //           --(static, launcher)--> lidar_frame
        frame_id_          = declare_parameter<std::string>("frame_id", "world");
        body_frame_        = declare_parameter<std::string>("body_frame", "base");
        lidar_frame_       = declare_parameter<std::string>("lidar_frame", "livox_frame");

        // ----------------
        // TF
        wait_for_tf_          = declare_parameter<bool>("wait_for_tf", true);   // gate /start_collection
        // How long a scan may wait for its pose to arrive. The listener runs
        // on its own thread, so this blocks only this callback.
        tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.1);
        // History depth. Must exceed the worst scan-to-pose lag; tf2's default
        // is 10 s, which is plenty here.
        tf_buffer_sec_         = declare_parameter<double>("tf_buffer_sec", 10.0);

        // ----------------
        // DRONE TRAJECTORY (world -> drone_frame, read from TF)
        // Needs something publishing that TF, e.g. mocap_tf_broadcaster.
        publish_trajectory_    = declare_parameter<bool>("publish_trajectory", true);
        drone_frame_           = declare_parameter<std::string>("drone_frame", "");  // "" -> body_frame
        trajectory_topic_      = declare_parameter<std::string>("trajectory_topic", "/drone_trajectory");
        trajectory_period_sec_ = declare_parameter<double>("trajectory_period_sec", 0.05);  // 20 Hz
        trajectory_min_dist_   = declare_parameter<double>("trajectory_min_dist", 0.005);   // m, 0 keeps every sample
        trajectory_max_points_ = declare_parameter<int>("trajectory_max_points", 20000);    // <= 0 unbounded

        // Offset from the drone to the middle of the fitted trunk axis,
        // in frame_id_. Published only while a cylinder fit is current.
        publish_offset_     = declare_parameter<bool>("publish_offset", true);
        offset_topic_       = declare_parameter<std::string>("offset_topic", "/processed/trunk_offset");
        offset_period_sec_  = declare_parameter<double>("offset_period_sec", 0.05);   // 20 Hz

        if (drone_frame_.empty()) drone_frame_ = body_frame_;

        // Buffer caches every transform heard on /tf and /tf_static, keeping
        // tf_buffer_sec_ of history so a stamped lookup can interpolate.
        // TransformListener subscribes to those topics on its OWN thread, so
        // the buffer keeps filling while cloudCallback blocks on a lookup.
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(
            get_clock(), tf2::durationFromSec(tf_buffer_sec_));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

                // ----------------
        // DESKEW
        // A Livox frame spans ~100 ms (measured 99.56 ms). One pose for all
        // of it smears the cloud by speed x 100 ms. Buckets cut that by ~10x.
        deskew_         = declare_parameter<bool>("deskew", true);
        deskew_buckets_ = declare_parameter<int>("deskew_buckets", 10);

        // MEMORIES
        save_path_ = declare_parameter<std::string>("save_path","");       
        mesh_path_ = declare_parameter<std::string>("mesh_path",""); 
        normals_save_path_  = declare_parameter<std::string>("normals_save_path", "");
        locked_targets_path_ = (std::filesystem::path(std::getenv("HOME")) 
                                /"vision_ws_outputs/normals/locked_target.yaml").string();

        output_dir_ = declare_parameter<std::string>(
            "output_dir",
            (std::filesystem::path(std::getenv("HOME")) / "vision_ws_outputs").string());

        locked_targets_path_ = declare_parameter<std::string>(
            "locked_target_path",
            (std::filesystem::path(std::getenv("HOME"))
                / "vision_ws_outputs" / "locked_target.yaml").string());
        
        // ----------------
        // OPERATIONS
        generate_path_           = declare_parameter<bool>("generate_path", false);
        path_yaml_               =declare_parameter<std::string>("path_yaml", "");
        normal_merge_radius_     = declare_parameter<double>("normal_merge_radius", 0.01);
        snapshot_period_sec_ = declare_parameter<double>("snapshot_period_sec", 5.0);


        // ----------------
        // PER-SCAN CROP SECTOR, in the LiDAR frame.
        //
        // Spherical coordinates about the sensor origin: a wedge in
        // azimuth, a wedge in elevation, and a shell between two radii.
        // This is the sensor's own coordinate system, so a window set
        // here maps directly onto what the Mid-360 can physically see.
        //
        //   azimuth   = atan2(y, x)            0 deg = +x boresight,
        //                                     +90 deg = +y, wraps at 180
        //   elevation = angle above the xy plane,  +90 deg = +z
        //   range     = |p|, true radial distance (not axial)
        //
        // Mid-360 hardware limits, for reference when choosing these:
        //   azimuth    360 deg (no limit)
        //   elevation  -7 to +52 deg
        //   range      0.1 m close-proximity blind zone
        // Asking for more than the sensor provides is harmless; it just
        // means the limit never binds.
        //
        // fov_min_range also does the job the old min_range parameter
        // did: the Mid-360 reports no-return beams as (0,0,0), and any
        // positive inner radius drops them.
        fov_min_range_        = declare_parameter<double>("fov_min_range", 0.10);
        fov_max_range_        = declare_parameter<double>("fov_max_range", 0.60);
        fov_min_azimuth_deg_  = declare_parameter<double>("fov_min_azimuth_deg", -20.0);
        fov_max_azimuth_deg_  = declare_parameter<double>("fov_max_azimuth_deg",  20.0);
        fov_min_elevation_deg_ = declare_parameter<double>("fov_min_elevation_deg", -20.0);
        fov_max_elevation_deg_ = declare_parameter<double>("fov_max_elevation_deg",  20.0);

        updateFovDerived();

        // Final crop box, applied in the WORLD frame to snapshots and the
        // final map. The world origin is the mocap origin, not the sensor,
        // so this is OFF by default.
        apply_final_bounds_ = declare_parameter<bool>("apply_final_bounds", false);
        final_min_x_ = declare_parameter<double>("final_min_x",  0.1);
        final_max_x_ = declare_parameter<double>("final_max_x",  0.5);
        final_min_y_ = declare_parameter<double>("final_min_y", -0.5);
        final_max_y_ = declare_parameter<double>("final_max_y",  0.5);
        final_min_z_ = declare_parameter<double>("final_min_z", -1.0);
        final_max_z_ = declare_parameter<double>("final_max_z",  1.0);

        voxel_leaf_size_        = declare_parameter<double>("voxel_leaf_size", 0.003);
        duplicate_distance_     = declare_parameter<double>("duplicate_distance", 0.001);
        
        // SOR denoising parameters
        sor_mean_k_             = declare_parameter<int>("sor_mean_k", 1000);
        sor_stddev_mult_        = declare_parameter<double>("sor_stddev_mult", 10.0);

        // ----------------
        // CLUSTER REMOVAL
        // The map is clustered on a VOXELISED COPY, never on global_map_
        // itself: Euclidean clustering is O(n log n) with a big constant,
        // and at duplicate_distance_ = 1 mm the real map is far too dense
        // to cluster inside a service callback.
        cluster_voxel_        = declare_parameter<double>("cluster_voxel", 0.01);
        // Two points join the same cluster if they are within this of each
        // other. Must exceed cluster_voxel_, or every voxel is its own
        // cluster. Rule of thumb: the widest real gap inside the trunk
        // surface, and smaller than the gap to the nearest other object.
        cluster_tolerance_    = declare_parameter<double>("cluster_tolerance", 0.05);
        cluster_min_points_   = declare_parameter<int>("cluster_min_points", 50);
        cluster_max_points_   = declare_parameter<int>("cluster_max_points", 10000000);
        // A click further than this from any clustered point is ignored,
        // so a stray click on empty space cannot delete anything.
        cluster_click_radius_ = declare_parameter<double>("cluster_click_radius", 0.10);
        clusters_topic_       = declare_parameter<std::string>("clusters_topic", "/processed/clusters");
        cluster_status_topic_ = declare_parameter<std::string>("cluster_status_topic", "/processed/cluster_status");

        // Cylinder RANSAC parameters
        max_ransac_iterations_        = declare_parameter<int>("max_ransac_iterations", 1000);
        min_ransac_radius_            = declare_parameter<double>("min_radius", 0.005);
        max_ransac_radius_            = declare_parameter<double>("max_radius", 0.15);
        ransac_probability_           = declare_parameter<double>("ransac_probability", 0.95);


        // Create new global_map
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        // -------------------------
        // SUBCRIBERS

        // Default: listening to "/dlio/odom_node/pointcloud/deskewed"
        // SensorDataQoS: prioritises receiving recent sensor data
        //                 rather than guaranteeing every single message arrives
        //                  (recommended for D-LIO)
        // bind: when a point cloud arrives, call cloudCallback() function

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&MapPipeline::cloudCallback, this, std::placeholders::_1));

        click_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            clicked_topic_, rclcpp::SensorDataQoS(),
            std::bind(&MapPipeline::onClickedPoint, this, std::placeholders::_1));
        
        // -------------------------------
        // PUBLISHERS

        // QoS(1): publisher has a q depth of 1 - ROS2 only needs to retain 
        //          approx one message for delivery
        // transient_local(): publisher keeps the most recent message available
        //          for subscribers taht connect later. 
        // CAUTION: publisher and subscriber QoS must be compatible!

        global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            global_map_topic_, rclcpp::QoS(1).transient_local());

        processed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            processed_topic_, rclcpp::QoS(1).transient_local());

        targets_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            targets_topic_ , rclcpp::QoS(1).transient_local());

        targets_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        targets_markers_topic_, rclcpp::QoS(1).transient_local());

        mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            mesh_topic_, rclcpp::QoS(1).transient_local());

        rim_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            rim_topic_, rclcpp::QoS(1).transient_local());

        cylinder_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            cylinder_marker_topic_, rclcpp::QoS(1).transient_local());

        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            path_topic_, rclcpp::QoS(1).transient_local());

        // Colour-coded clusters for RViz, and a one-line status the GUI
        // subscribes to. Both transient_local: the panel and RViz latch on
        // whenever they connect, which is how the panel learns that a click
        // has landed without polling for it.
        clusters_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            clusters_topic_, rclcpp::QoS(1).transient_local());

        cluster_status_pub_ = create_publisher<std_msgs::msg::String>(
            cluster_status_topic_, rclcpp::QoS(1).transient_local());

        // Drone trajectory. transient_local so RViz sees the path so far
        // even if it (re)connects mid-flight.
        trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
            trajectory_topic_, rclcpp::QoS(1).transient_local());
        
        // Sample the drone pose on a fixed period. Polling TF is simpler than
        // a second subscriber, and works with whatever publishes the TF
        // (mocap bridge now, a flight-controller odometry bridge later).
        if (publish_trajectory_)
        {
            trajectory_.header.frame_id = frame_id_;

            trajectory_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(trajectory_period_sec_)),
                std::bind(&MapPipeline::sampleTrajectory, this));
        }

        // Drone -> trunk axis midpoint. Plain volatile QoS: this is a live
        // measurement, and a stale one is worse than none.
        offset_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            offset_topic_, rclcpp::QoS(10));

        if (publish_offset_)
        {
            offset_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(offset_period_sec_)),
                std::bind(&MapPipeline::publishTrunkOffset, this));
        }

        // Poll until frame_id <- lidar_frame resolves, then stop.
        // canTransform() never throws; it just answers yes/no.
        tf_wait_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                if (tf_buffer_->canTransform(frame_id_, lidar_frame_, tf2::TimePointZero))
                {
                    have_tf_ = true;
                    logExtrinsic();
                    tf_wait_timer_->cancel();
                    RCLCPP_INFO(get_logger(), "TF chain live — /start_collection is now accepted.");
                    return;
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Still waiting for TF '%s' -> '%s'. Are the mocap bridge and "
                    "static_transform_publisher both running?",
                    frame_id_.c_str(), lidar_frame_.c_str());
            });

        // Services for start and end of cloud accumulation
        start_collection_srv_ = create_service<std_srvs::srv::Trigger>(
            "start_collection",
            std::bind(&MapPipeline::startCollection, this,
                      std::placeholders::_1, std::placeholders::_2));

        stop_collection_srv_ = create_service<std_srvs::srv::Trigger>(
            "stop_collection",
            std::bind(&MapPipeline::stopCollection, this,
                      std::placeholders::_1, std::placeholders::_2));

        clear_map_srv_ = create_service<std_srvs::srv::Trigger>(
            "clear_map",
            std::bind(&MapPipeline::clearMapService, this,
                      std::placeholders::_1, std::placeholders::_2));

        
        // Normal Selection Services
            
        load_locked_srv_ = create_service<std_srvs::srv::Trigger>(
            "load_locked_target",
            std::bind(&MapPipeline::loadLockedTargetService, this,
                      std::placeholders::_1, std::placeholders::_2));

        undo_selection_srv_ =
            this->create_service<std_srvs::srv::Trigger>(
                "undo_normal_selection",
                std::bind(
                    &MapPipeline::undoNormalSelection,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        clear_normals_srv_ =
            this->create_service<std_srvs::srv::Trigger>(
                "clearNormals",
                std::bind(
                    &MapPipeline::clearNormals,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        
        dump_normals_srv_ = create_service<std_srvs::srv::Trigger>(
            "dump_normals",
            std::bind(&MapPipeline::dumpNormals, 
                      this,
                      std::placeholders::_1, 
                      std::placeholders::_2));

        save_normals_srv_ = create_service<std_srvs::srv::Trigger>(
            "save_normals",
            std::bind(&MapPipeline::saveNormalsService, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Path services
        load_path_srv_ =
            this->create_service<std_srvs::srv::Trigger>(
                "load_and_publish_path",
                std::bind(
                    &MapPipeline::loadAndPublishPathService,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        clear_path_srv_ = create_service<std_srvs::srv::Trigger>(
            "clear_path",
            std::bind(&MapPipeline::clearPathService, this,
                      std::placeholders::_1, std::placeholders::_2));

        clear_trajectory_srv_ = create_service<std_srvs::srv::Trigger>(
            "clear_trajectory",
            std::bind(&MapPipeline::clearTrajectoryService, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Cluster removal: arm -> click in RViz -> confirm / cancel.
        cluster_map_srv_ = create_service<std_srvs::srv::Trigger>(
            "cluster_map",
            std::bind(&MapPipeline::clusterMapService, this,
                      std::placeholders::_1, std::placeholders::_2));

        confirm_cluster_srv_ = create_service<std_srvs::srv::Trigger>(
            "confirm_cluster_removal",
            std::bind(&MapPipeline::confirmClusterRemovalService, this,
                      std::placeholders::_1, std::placeholders::_2));

        cancel_cluster_srv_ = create_service<std_srvs::srv::Trigger>(
            "cancel_cluster_removal",
            std::bind(&MapPipeline::cancelClusterRemovalService, this,
                      std::placeholders::_1, std::placeholders::_2));

        publishClusterStatus("idle");
        

        RCLCPP_INFO(get_logger(),
            "map_pip_tf ready: cloud '%s', map frame '%s', lidar frame '%s', "
            "trajectory %s ('%s' -> %s). Waiting for /start_collection.",
            cloud_topic_.c_str(), frame_id_.c_str(), lidar_frame_.c_str(),
            publish_trajectory_ ? "on" : "off",
            drone_frame_.c_str(), trajectory_topic_.c_str());

    }

    // destructor + early termination: two ways to save: normal completion / early shutdown
    // destructor: before MapPipeLIne object disappears, do a final cleanup/ finalisation work. 
    ~MapPipeline()
        {
            // Fallback: Ctrl+C mid-run. If finalizeRun() already ran, saved_
            // is true and this is a no-op.
            if (collecting_) saveMap();
        }

private:

    // Explicit-timeout form. Bucket lookups pass 0: the whole-scan lookup
    // already waited, so the data is in the buffer and blocking 10 times
    // over would stall the callback for a second.
    bool lookupLidarPose(const std::string &source_frame,
                         const rclcpp::Time &stamp,
                         double timeout_sec,
                         Eigen::Isometry3d &T_world_lidar,
                         std::string &reason)
    {
        try
        {
            const geometry_msgs::msg::TransformStamped tf_msg =
                tf_buffer_->lookupTransform(
                    frame_id_,          // target: express points in the map frame
                    source_frame,       // source: the frame the scan was taken in
                    stamp,              // AT THE SCAN'S OWN TIME, not "latest"
                    tf2::durationFromSec(timeout_sec));
            T_world_lidar = tf2::transformToEigen(tf_msg);
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            reason = ex.what();
            return false;
        }
    }

    bool lookupLidarPose(const std::string &source_frame,
                         const rclcpp::Time &stamp,
                         Eigen::Isometry3d &T_world_lidar,
                         std::string &reason)
    {
        return lookupLidarPose(source_frame, stamp, tf_lookup_timeout_sec_,
                               T_world_lidar, reason);
    }

    // ---- Deskew ----
    // Splits the scan into time buckets by each point's own timestamp, looks
    // up a pose per bucket, and transforms each bucket with its own pose.
    // Filtering stays in the LiDAR frame, exactly as the single-pose path.
    // Returns false (and leaves world_scan alone) if deskewing isn't possible,
    // so the caller can fall back.
    bool deskewScan(const sensor_msgs::msg::PointCloud2 &msg,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &world_scan,
                    double &skew_mm,
                    std::string &reason)
    {
        const auto *t_field = findField(
            msg, {"timestamp", "t", "time", "time_stamp", "offset_time"});
        if (!t_field) { reason = "no per-point time field"; return false; }

        const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;
        if (n == 0) { reason = "empty cloud"; return false; }

        const rclcpp::Time scan_stamp(msg.header.stamp, RCL_ROS_TIME);

        // Units, decided once from the first value. Livox PointCloud2 mode
        // writes absolute epoch nanoseconds as float64 (~1.79e18).
        const double v0 = readFieldAsDouble(msg, *t_field, 0);
        if (!std::isfinite(v0)) { reason = "unsupported time datatype"; return false; }

        double scale; bool absolute;
        if      (v0 > 1e15) { scale = 1e-9; absolute = true;  }   // epoch ns
        else if (v0 > 1e6)  { scale = 1e-9; absolute = false; }   // relative ns
        else                { scale = 1.0;  absolute = false; }   // relative s

        const double base = absolute ? scan_stamp.seconds() : 0.0;
        auto rel_time = [&](std::size_t i) {
            return readFieldAsDouble(msg, *t_field, i) * scale - base;
        };

        // Pass 1: the span the cloud covers. Points are NOT in time order on
        // the Mid-360 (non-repetitive pattern), so this must scan them all.
        double t_lo = std::numeric_limits<double>::max();
        double t_hi = std::numeric_limits<double>::lowest();
        for (std::size_t i = 0; i < n; ++i)
        {
            const double t = rel_time(i);
            if (!std::isfinite(t)) continue;
            t_lo = std::min(t_lo, t);
            t_hi = std::max(t_hi, t);
        }
        const double span = t_hi - t_lo;
        if (!(span > 1e-4)) { reason = "time span ~0"; return false; }

        // Wait ONCE, for the pose covering the end of the scan. If that
        // arrives, every earlier bucket is already buffered.
        Eigen::Isometry3d probe;
        if (!lookupLidarPose(msg.header.frame_id,
                             scan_stamp + rclcpp::Duration::from_seconds(t_hi),
                             tf_lookup_timeout_sec_, probe, reason))
            return false;

        const int nb = std::max(1, deskew_buckets_);
        const double dt = span / nb;
        std::vector<Eigen::Isometry3d> pose(nb);
        std::vector<char> pose_ok(nb, -1);      // -1 untried, 0 failed, 1 good

        auto bucket_pose = [&](int b) -> bool {
            if (pose_ok[b] >= 0) return pose_ok[b] == 1;
            const rclcpp::Time t = scan_stamp + rclcpp::Duration::from_seconds(
                t_lo + (b + 0.5) * dt);          // bucket centre
            std::string why;
            pose_ok[b] = lookupLidarPose(msg.header.frame_id, t, 0.0,
                                         pose[b], why) ? 1 : 0;
            return pose_ok[b] == 1;
        };

        // Pass 2: filter in the LiDAR frame, then transform per bucket.
        sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

        world_scan->clear();
        world_scan->reserve(n / 4);

        for (std::size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z)
        {
            const float x = *it_x, y = *it_y, z = *it_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            // also drops the (0,0,0) no-return points, via fov_min_range
            if (!inFov(x, y, z)) continue;

            const double t = rel_time(i);
            if (!std::isfinite(t)) continue;

            // clamp: the latest point sits exactly on the upper edge
            int b = static_cast<int>((t - t_lo) / dt);
            b = std::clamp(b, 0, nb - 1);
            if (!bucket_pose(b)) { ++deskew_pose_misses_; continue; }

            const Eigen::Vector3d p_w = pose[b] * Eigen::Vector3d(x, y, z);
            world_scan->push_back(pcl::PointXYZ(
                static_cast<float>(p_w.x()),
                static_cast<float>(p_w.y()),
                static_cast<float>(p_w.z())));
        }

        // Sensor travel across the scan: the error just removed.
        int first = -1, last = -1;
        for (int b = 0; b < nb; ++b)
            if (pose_ok[b] == 1) { if (first < 0) first = b; last = b; }
        skew_mm = (first >= 0 && last > first)
            ? (pose[last].translation() - pose[first].translation()).norm() * 1e3
            : 0.0;

        if (world_scan->empty()) { reason = "all points filtered or unposed"; return false; }
        return true;
    }

    // One-off log of the static extrinsic actually in TF, so the boot log
    // records which calibration this run used.
    void logExtrinsic()
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        try
        {
            tf_msg = tf_buffer_->lookupTransform(
                body_frame_, lidar_frame_, tf2::TimePointZero);  // static: any time
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not read %s -> %s: %s",
                body_frame_.c_str(), lidar_frame_.c_str(), ex.what());
            return;
        }

        const Eigen::Isometry3d T = tf2::transformToEigen(tf_msg);

        // R = Rz(yaw) * Ry(pitch) * Rx(roll), same convention as
        // static_transform_publisher, so the log matches the launcher's values.
        const Eigen::Vector3d t = T.translation();
        const Eigen::Matrix3d R = T.linear();
        const double roll  = std::atan2(R(2, 1), R(2, 2));
        const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
        const double yaw   = std::atan2(R(1, 0), R(0, 0));
        RCLCPP_INFO(get_logger(),
            "Static extrinsic %s -> %s: xyz=(%.4f, %.4f, %.4f) rpy=(%.4f, %.4f, %.4f)",
            body_frame_.c_str(), lidar_frame_.c_str(),
            t.x(), t.y(), t.z(), roll, pitch, yaw);
    }

    // ---- Drone trajectory ----
    // One timer tick: read world -> drone_frame from TF, append it to a
    // nav_msgs::msg::Path and republish. The whole path is resent each time;
    // that is what RViz's Path display expects, and at 20 Hz it is cheap.
    // Vector from the drone to the middle of the fitted trunk axis, in
    // frame_id_. Same TF poll as sampleTrajectory(): whatever publishes
    // frame_id_ -> drone_frame_ drives this too.
    void publishTrunkOffset()
    {
        if (!has_cylinder_) return;   // no fit yet, or cleared

        geometry_msgs::msg::TransformStamped tf_msg;
        try
        {
            tf_msg = tf_buffer_->lookupTransform(
                frame_id_, drone_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Offset: no TF '%s' -> '%s' (%s).",
                frame_id_.c_str(), drone_frame_.c_str(), ex.what());
            return;
        }

        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.frame_id = frame_id_;
        msg.header.stamp    = tf_msg.header.stamp;   // the pose this was measured against
        msg.vector.x = last_cylinder_mid_.x() - tf_msg.transform.translation.x;
        msg.vector.y = last_cylinder_mid_.y() - tf_msg.transform.translation.y;
        msg.vector.z = last_cylinder_mid_.z() - tf_msg.transform.translation.z;
        offset_pub_->publish(msg);
    }

    void sampleTrajectory()
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        try
        {
            // Latest available. Unlike the scans, this is only drawn for the
            // operator, so "newest pose" is the right answer; asking for now()
            // would race the bridge and throw ExtrapolationException.
            tf_msg = tf_buffer_->lookupTransform(
                frame_id_, drone_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Trajectory: no TF '%s' -> '%s' (%s). Is the pose bridge running?",
                frame_id_.c_str(), drone_frame_.c_str(), ex.what());
            return;
        }

        // The buffer keeps serving the last transform after the publisher
        // stops, so an unchanged stamp means no new pose, not a stationary drone.
        const rclcpp::Time stamp(tf_msg.header.stamp, RCL_ROS_TIME);
        if (!trajectory_.poses.empty() && stamp == last_trajectory_stamp_) return;
        last_trajectory_stamp_ = stamp;

        geometry_msgs::msg::PoseStamped pose;
        pose.header = tf_msg.header;                              // frame_id_ + the TF's own stamp
        pose.pose.position.x = tf_msg.transform.translation.x;    // a translation is a Vector3,
        pose.pose.position.y = tf_msg.transform.translation.y;    // a position is a Point: same
        pose.pose.position.z = tf_msg.transform.translation.z;    // fields, different types, so
        pose.pose.orientation = tf_msg.transform.rotation;        // copy field by field

        // Drop samples that have not moved, so hovering does not fill the
        // path with thousands of coincident poses.
        if (!trajectory_.poses.empty() && trajectory_min_dist_ > 0.0)
        {
            const auto &prev = trajectory_.poses.back().pose.position;
            const double dx = pose.pose.position.x - prev.x;
            const double dy = pose.pose.position.y - prev.y;
            const double dz = pose.pose.position.z - prev.z;
            if (dx * dx + dy * dy + dz * dz <
                trajectory_min_dist_ * trajectory_min_dist_) return;
        }

        if (trajectory_.poses.empty())
        {
            RCLCPP_INFO(get_logger(),
                "Trajectory: first drone pose at (%.3f, %.3f, %.3f) in '%s'.",
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                frame_id_.c_str());
        }

        trajectory_.poses.push_back(pose);

        // Sliding window on length: oldest poses drop off the front.
        if (trajectory_max_points_ > 0 &&
            static_cast<int>(trajectory_.poses.size()) > trajectory_max_points_)
        {
            trajectory_.poses.erase(trajectory_.poses.begin());
        }

        trajectory_.header.stamp = now();
        trajectory_pub_->publish(trajectory_);
    }

    void clearTrajectoryService(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        trajectory_.poses.clear();
        trajectory_.header.stamp = now();
        trajectory_pub_->publish(trajectory_);   // replaces the latched copy too

        RCLCPP_INFO(get_logger(), "Cleared drone trajectory.");
        response->success = true;
        response->message = "Trajectory cleared.";
    }

    // World-frame crop shared by writeSnapshot() and finalizeRun(), so the
    // live mesh and the final mesh always see the same volume.
    // Returns a copy; with apply_final_bounds=false it is just a copy.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropToFinalBounds(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
        if (!apply_final_bounds_) { *out = *in; return out; }

        out->reserve(in->size());
        for (const auto &p : in->points)
        {
            if (p.x >= final_min_x_ && p.x <= final_max_x_ &&
                p.y >= final_min_y_ && p.y <= final_max_y_ &&
                p.z >= final_min_z_ && p.z <= final_max_z_)
            {
                out->push_back(p);
            }
        }
        return out;
    }
    
    // COLLECTION SERVICES
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!collecting_) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr world_scan(
            new pcl::PointCloud<pcl::PointXYZ>);
        double skew_mm = 0.0;
        std::string reason;

        if (!deskew_ || !deskewScan(*msg, world_scan, skew_mm, reason))
        {
            if (deskew_)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Deskew unavailable (%s). Falling back to one pose per scan.",
                    reason.c_str());

            // ---- original single-pose path ----
            pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *scan);
            if (scan->empty()) return;

            pcl::PointCloud<pcl::PointXYZ>::Ptr bounded_scan(
                new pcl::PointCloud<pcl::PointXYZ>);
            bounded_scan->reserve(scan->size());

            for (const auto &p : scan->points)
                if (inFov(p.x, p.y, p.z)) bounded_scan->push_back(p);

            Eigen::Isometry3d T_world_lidar;
            if (!lookupLidarPose(msg->header.frame_id,
                                 rclcpp::Time(msg->header.stamp, RCL_ROS_TIME),
                                 T_world_lidar, reason))
            {
                ++scans_dropped_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Dropping scan (%zu dropped so far): %s. If this never clears, "
                    "check that the mocap bridge and the LiDAR agree on the ROS "
                    "clock, or raise tf_lookup_timeout_sec.",
                    scans_dropped_, reason.c_str());
                return;
            }

            pcl::transformPointCloud(*bounded_scan, *world_scan,
                                     T_world_lidar.cast<float>().matrix());
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr new_points(
        new pcl::PointCloud<pcl::PointXYZ>);

        if (global_map_->empty())
        {
            // First scan: everything is new
            *new_points = *world_scan;
        }
        else
        {
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(global_map_);

            std::vector<int> nearest_indices;
            std::vector<float> nearest_distances;

            for (const auto &point : world_scan->points)
            {
                nearest_indices.clear();
                nearest_distances.clear();

                if (kdtree.radiusSearch(
                        point,
                        duplicate_distance_,
                        nearest_indices,
                        nearest_distances,
                        1) == 0)
                {
                    // No existing point nearby
                    new_points->points.push_back(point);
                }
            }
        }

        // ---------------------------------------------------------
        // 6. Add only genuinely new points to global map
        // ---------------------------------------------------------

        *global_map_ += *new_points;

        RCLCPP_INFO(
            get_logger(),
            "Scan: %zu points | New: %zu | Map: %zu | skew: %.1f mm",
            world_scan->size(),
            new_points->size(),
            global_map_->size(),
            skew_mm);

        // create an empty ROS pointcloud2 msg
        sensor_msgs::msg::PointCloud2 out;

        // temporarily lock the map to convert from PCL pc to ROS pc
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            pcl::toROSMsg(*global_map_, out);
        }

        // Set up timestamp and frame stamp.
        out.header.stamp = msg->header.stamp; // gives published map a timestamp. could also use ->now()
        out.header.frame_id = frame_id_; // current frame is 'odom'. tells Rviz that global_map is in 'odom' coord. 
        global_map_pub_->publish(out); // send the map onto the topic oif global_map_pub
    }
        

        // Periodic while collecting: pick up the newest live_*.stl the mesh loop
    // has produced and swap it in, so click-to-select tracks the scan.
    // Cheap when nothing has changed — one directory scan, no file I/O.
    // Recompute what the per-point test needs, and reject a sector that
    // would pass nothing. A silently empty sector looks exactly like a TF
    // failure in the logs, and it feeds degenerate clouds downstream.
    void updateFovDerived()
    {
        if (fov_min_range_ < 0.0) fov_min_range_ = 0.0;
        if (fov_max_range_ <= fov_min_range_)
        {
            RCLCPP_WARN(get_logger(),
                "fov_max_range (%.3f) <= fov_min_range (%.3f) — the sector is "
                "empty. Forcing fov_max_range to fov_min_range + 0.1.",
                fov_max_range_, fov_min_range_);
            fov_max_range_ = fov_min_range_ + 0.1;
        }
        fov_min_range_sq_ = static_cast<float>(fov_min_range_ * fov_min_range_);
        fov_max_range_sq_ = static_cast<float>(fov_max_range_ * fov_max_range_);

        // ---- elevation ----
        // Elevation does not wrap, so min above max is simply an error.
        if (fov_min_elevation_deg_ > fov_max_elevation_deg_)
        {
            RCLCPP_WARN(get_logger(),
                "fov_min_elevation_deg (%.1f) > fov_max_elevation_deg (%.1f) — "
                "swapping them.",
                fov_min_elevation_deg_, fov_max_elevation_deg_);
            std::swap(fov_min_elevation_deg_, fov_max_elevation_deg_);
        }

        // The test below compares z against tan(elevation) * rho, which is
        // the same ordering as comparing the angles themselves but avoids
        // an asin per point. tan() is only monotonic on (-90, +90), and is
        // NEGATIVE past 90, which would silently invert the test — so a
        // request reaching either pole becomes "no elevation limit" instead.
        fov_el_full_ = (fov_min_elevation_deg_ <= -89.9 &&
                        fov_max_elevation_deg_ >=  89.9);
        if (!fov_el_full_)
        {
            const double lo = std::clamp(fov_min_elevation_deg_, -89.9, 89.9);
            const double hi = std::clamp(fov_max_elevation_deg_, -89.9, 89.9);
            fov_tan_el_min_ = static_cast<float>(std::tan(lo * M_PI / 180.0));
            fov_tan_el_max_ = static_cast<float>(std::tan(hi * M_PI / 180.0));
        }

        // ---- azimuth ----
        // Azimuth DOES wrap, so min above max is meaningful: it is a wedge
        // straddling +-180. The span is measured counter-clockwise from
        // min to max, so -170 -> 170 is the wide 340 deg wedge and
        // 170 -> -170 is the narrow 20 deg one. A raw difference of a
        // full turn or more (e.g. -180 -> 180) means the whole circle;
        // that has to be caught before the fmod, which would fold it to 0
        // and pass nothing.
        const double raw = fov_max_azimuth_deg_ - fov_min_azimuth_deg_;
        if (std::abs(raw) >= 360.0)
        {
            fov_az_full_ = true;
            fov_az_span_ = 360.0f;
        }
        else
        {
            double span = std::fmod(raw, 360.0);
            if (span < 0.0) span += 360.0;
            fov_az_full_ = (span >= 359.999);
            fov_az_span_ = static_cast<float>(span);
        }
        fov_az_min_f_ = static_cast<float>(fov_min_azimuth_deg_);
    }

    // True when (x, y, z) in the LIDAR frame is inside the crop sector.
    //
    // Ordered cheapest test first, because this is the hottest loop in
    // the node: ~20k points per scan at 10 Hz. Range costs nothing but
    // multiplies, elevation costs one sqrt, azimuth costs an atan2 and
    // is skipped entirely when the whole circle is wanted.
    inline bool inFov(float x, float y, float z) const
    {
        const float rho2 = x * x + y * y;
        const float r2   = rho2 + z * z;
        if (r2 < fov_min_range_sq_ || r2 > fov_max_range_sq_) return false;

        if (!fov_el_full_)
        {
            // rho >= 0, so multiplying through by it preserves the
            // inequality: z/rho >= tan(lo)  <=>  z >= tan(lo)*rho.
            const float rho = std::sqrt(rho2);
            if (z < fov_tan_el_min_ * rho) return false;
            if (z > fov_tan_el_max_ * rho) return false;
        }

        if (fov_az_full_) return true;

        // How far counter-clockwise the point sits from the lower edge.
        // Folding into [0, 360) first means a wedge across +-180 needs no
        // special case.
        constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);
        float d = std::fmod(std::atan2(y, x) * RAD2DEG - fov_az_min_f_, 360.0f);
        if (d < 0.0f) d += 360.0f;
        return d <= fov_az_span_;
    }

    void checkForNewMesh()
    {
        if (!collecting_) return;

        std::string newest;
        try
        {
            const auto dir = run_dir_ / "meshes";
            for (const auto& entry : std::filesystem::directory_iterator(dir))
            {
                const auto name = entry.path().filename().string();
                // live_NNNNN.stl only. The mesh loop writes via a dot-prefixed
                // temp name, so partial files can never match this.
                if (name.rfind("live_", 0) == 0 && entry.path().extension() == ".stl")
                {
                    // live_NNNNN.stl is built from snapshot_NNNNN.pcd. Anything
                    // numbered below the floor predates the last /clear_map.
                    int index = -1;
                    try { index = std::stoi(name.substr(5)); } catch (...) {}
                    if (index < live_mesh_floor_) continue;

                    if (name > std::filesystem::path(newest).filename().string() || newest.empty())
                        newest = entry.path().string();
                }
            }
        }
        catch (const std::exception&) { return; }   // dir not ready yet

        if (newest.empty() || newest == loaded_mesh_path_) return;

        if (loadAndPublishMesh(newest, /*wait_for_file=*/false))
        {
            loaded_mesh_path_ = newest;
            RCLCPP_INFO(get_logger(), "Live mesh: %s (%zu faces)",
                        std::filesystem::path(newest).filename().c_str(),
                        mesh_centroids_->size());

            std_msgs::msg::Header hdr;
            hdr.frame_id = frame_id_;
            hdr.stamp = now();

            t.start();
            fitAndPublishCylinder(mesh_centroids_, mesh_face_normals_, hdr);
            RCLCPP_INFO(get_logger(), "[TIMER] Live cylinder fit: %.2f ms", t.stop_ms());
        }
    }

        // Periodic while collecting: write a filtered copy of the accumulated
    // cloud to maps/ as snapshot_NNNNN.pcd. Never mutates global_map_ —
    // accumulation continues underneath, untouched.
    void writeSnapshot()
    {
        if (!collecting_) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr snap;   // filled by cropToFinalBounds() below
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (global_map_->size() < 100) return;   // too sparse to mesh
            snap = cropToFinalBounds(global_map_);   // world-frame crop (no-op if disabled)
        }
        if (snap->size() < 100) return;

        // Voxelise only — SOR is deliberately skipped here. It's the
        // expensive step and the live mesh doesn't need it.
        pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>);
        {
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(snap);
            vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            vg.filter(*sampled);
        }

        char buf[32];
        std::snprintf(buf, sizeof(buf), "snapshot_%05d", snapshot_count_);
        const auto final_path = run_dir_ / "maps" / (std::string(buf) + ".pcd");
        const auto tmp_path   = run_dir_ / "maps" / (std::string(buf) + ".pcd.tmp");

        t.start();
        if (pcl::io::savePCDFileBinary(tmp_path.string(), *sampled) < 0)
        {
            RCLCPP_WARN(get_logger(), "Snapshot %d: write failed.", snapshot_count_);
            std::filesystem::remove(tmp_path);
            return;
        }

        // Atomic publish: rename(2) within one filesystem is a single step,
        // so the mesh loop never opens a partially written PCD.
        std::error_code ec;
        std::filesystem::rename(tmp_path, final_path, ec);
        if (ec)
        {
            RCLCPP_WARN(get_logger(), "Snapshot %d: rename failed (%s).",
                        snapshot_count_, ec.message().c_str());
            std::filesystem::remove(tmp_path);
            return;
        }

        RCLCPP_INFO(get_logger(), "Snapshot %05d: %zu points (%.1f ms)",
                    snapshot_count_, sampled->size(), t.stop_ms());
        ++snapshot_count_;
    }

        // ---- runs once per run, shortly after /stop_collection ----
    void finalizeRun()
    {
        finalize_timer_->cancel();   // one-shot: never fire again

        // collecting_ was already cleared by stopCollection(). cloud_sub_
        // stays alive (gated by collecting_) so the next /start_collection
        // can reuse it without re-subscribing.

        RCLCPP_INFO(get_logger(),
            "TF summary: %zu scans dropped for want of a transform.",
            scans_dropped_);

        // BOUND: optional final crop in the WORLD frame.
        {
            auto bounded_global = cropToFinalBounds(global_map_);
            if (bounded_global->empty())
            {
                RCLCPP_ERROR(get_logger(),
                    "Final crop removed every point — box is in the wrong place for "
                    "this world origin. Keeping the uncropped map.");
            }
            else
            {
                global_map_ = bounded_global;
            }
        }

        // DENOISE: denoise the entire cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr denoised (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(global_map_);
        sor.setMeanK(sor_mean_k_);
        sor.setStddevMulThresh(sor_stddev_mult_);
        sor.filter(*denoised);
        if (denoised->empty())
        {
            RCLCPP_WARN(get_logger(), "Cloud empty after SOR — check sor_mean_k / sor_stddev_mult.");
            return;
        }

        global_map_ = denoised;

        // locking map and get map size
        size_t final_count = 0;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            final_count = global_map_->size();
        }
        RCLCPP_INFO(get_logger(),
            "Collection complete (%.1fs). Final map: %zu points. Running pipeline...",
            (now() - run_start_time_).seconds(), final_count);

        saveMap();

        // Publish the processed global map for RViz
        {
            sensor_msgs::msg::PointCloud2 out;
            pcl::toROSMsg(*global_map_, out);
            out.header.frame_id = frame_id_;
            out.header.stamp = now();
            last_processed_msg_ = out;
            processed_pub_->publish(out);
        }

        // Load mesh — face normals are the source for click-to-select and cylinder fit
        if (!loadAndPublishMesh(mesh_path_))
        {
            RCLCPP_ERROR(get_logger(),
                "Mesh load failed — click-to-select will not work this session.");
        }
        else
        {
            // Fit cylinder using mesh face centroids + face normals
            std_msgs::msg::Header hdr;
            hdr.frame_id = frame_id_;
            hdr.stamp = now();

            t.start();
            fitAndPublishCylinder(mesh_centroids_, mesh_face_normals_, hdr);
            RCLCPP_INFO(get_logger(), "[TIMER] Cylinder Fitting: %.2f ms", t.stop_ms());
        }

        pipeline_done_ = true;

        // Keepalive: republish processed cloud at 1 Hz for late-joining RViz subscribers
        keepalive_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            if (!pipeline_done_) return;
            processed_pub_->publish(last_processed_msg_);
        });

        RCLCPP_INFO(get_logger(),
            "Pipeline complete. Mesh loaded — click faces in RViz2 to select normals, or Ctrl+C to exit.");
    }

    void startCollection(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (collecting_)
        {
            res->success = false;
            res->message = "Already collecting.";
            return;
        }

        if (wait_for_tf_ && !have_tf_)
        {
            res->success = false;
            res->message = "No TF yet from " + frame_id_ + " to " + lidar_frame_ + ".";
            return;
        }

        // The control panel changes these between runs with a parameter
        // set; re-read them here so a new run uses the current sector.
        fov_min_range_         = get_parameter("fov_min_range").as_double();
        fov_max_range_         = get_parameter("fov_max_range").as_double();
        fov_min_azimuth_deg_   = get_parameter("fov_min_azimuth_deg").as_double();
        fov_max_azimuth_deg_   = get_parameter("fov_max_azimuth_deg").as_double();
        fov_min_elevation_deg_ = get_parameter("fov_min_elevation_deg").as_double();
        fov_max_elevation_deg_ = get_parameter("fov_max_elevation_deg").as_double();
        updateFovDerived();

        apply_final_bounds_ = get_parameter("apply_final_bounds").as_bool();
        final_min_x_ = get_parameter("final_min_x").as_double();
        final_max_x_ = get_parameter("final_max_x").as_double();
        final_min_y_ = get_parameter("final_min_y").as_double();
        final_max_y_ = get_parameter("final_max_y").as_double();
        final_min_z_ = get_parameter("final_min_z").as_double();
        final_max_z_ = get_parameter("final_max_z").as_double();

        RCLCPP_INFO(get_logger(),
            "FOV sector: range [%.3f, %.3f] m, azimuth %s, elevation %s%s",
            fov_min_range_, fov_max_range_,
            fov_az_full_
                ? "full 360 deg"
                : (std::to_string(static_cast<int>(fov_min_azimuth_deg_)) + " to "
                 + std::to_string(static_cast<int>(fov_max_azimuth_deg_)) + " deg").c_str(),
            fov_el_full_
                ? "unlimited"
                : (std::to_string(static_cast<int>(fov_min_elevation_deg_)) + " to "
                 + std::to_string(static_cast<int>(fov_max_elevation_deg_)) + " deg").c_str(),
            apply_final_bounds_ ? "  (final bounds on)" : "");

        if (!openRunDirectory())
        {
            res->success = false;
            res->message = "Failed to create run directory.";
            return;
        }
        
        /* // DO NOT delete previous run map. 
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        }
        */

        saved_         = false;
        pipeline_done_ = false;
        if (keepalive_timer_) keepalive_timer_->cancel();

        run_start_time_ = now();
        scans_dropped_  = 0;
        collecting_     = true;

        snapshot_count_ = 0;
        live_mesh_floor_ = 0;

        // A selection armed against the previous map must not survive into
        // a new run: its indices point into a cloud that is about to change.
        resetClusterSelection();
        snapshot_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(snapshot_period_sec_)),
            std::bind(&MapPipeline::writeSnapshot, this));

        loaded_mesh_path_.clear();
        mesh_watch_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MapPipeline::checkForNewMesh, this));

        RCLCPP_INFO(get_logger(), "Collection started.");
        res->success = true;
        res->message = run_dir_.string();
    }

    void stopCollection(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (!collecting_)
        {
            res->success = false;
            res->message = "Not currently collecting.";
            return;
        }

        collecting_ = false;   // cloudCallback() drops scans from here on

        if (snapshot_timer_) snapshot_timer_->cancel();

        if (mesh_watch_timer_) mesh_watch_timer_->cancel();


        // Defer the heavy work (bound/SOR/save/mesh-load) to a one-shot
        // timer so this service returns straight away. Blocking here would
        // deadlock the caller: loadAndPublishMesh() waits on an STL that the
        // caller itself is responsible for generating.
        finalize_timer_ = create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&MapPipeline::finalizeRun, this));

        RCLCPP_INFO(get_logger(), "Collection stopped. Finalising...");
        res->success = true;
        res->message = "Collection stopped.";
    }

    void clearMapService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // finalizeRun() is queued on a 1 ms timer by /stop_collection and
        // cancels that timer first. Clearing in between would hand it an
        // empty map to save and mesh.
        if (finalize_timer_ && !finalize_timer_->is_canceled())
        {
            response->success = false;
            response->message = "Run is finalising. Try again once the final mesh is loaded.";
            return;
        }

        size_t cleared_points = 0;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            cleared_points = global_map_->size();
            global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        }

        const size_t cleared_normals = selected_normals_.size();
        selected_normals_.clear();

        // Every click/normal service checks
        //   (!pipeline_done_ && loaded_mesh_path_.empty())
        // so these two refuse them until the next live mesh is loaded.
        // pipeline_done_ = false also stops the keepalive republish.
        pipeline_done_ = false;
        loaded_mesh_path_.clear();
        mesh_centroids_.reset();
        mesh_face_normals_.reset();
        mesh_tree_.reset();
        has_cylinder_ = false;

        // Live meshes are numbered after their snapshot; the next snapshot
        // written is the first one built from the cleared map.
        live_mesh_floor_ = snapshot_count_;

        resetClusterSelection();     // cached cluster indices are now stale
        clearRVizVisualizations();   // clouds, markers, mesh, path
        publishSelectedNormals();    // empty PoseArray + DELETEALL

        std::ostringstream msg;
        msg << "Cleared " << cleared_points << " points, "
            << cleared_normals << " normal(s), mesh and path.";
        if (collecting_)
        {
            msg << " Still collecting; live meshes resume from snapshot "
                << std::setw(5) << std::setfill('0') << live_mesh_floor_ << ".";
        }

        RCLCPP_INFO(get_logger(), "clear_map: %s", msg.str().c_str());
        response->success = true;
        response->message = msg.str();
    }

        // Create run_<TIMESTAMP>/ with its subfolders, derive this run's output
    // paths, and repoint <output_dir>/latest at it.
    bool openRunDirectory()
    {
        const auto now_c = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now());
        std::ostringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
        run_stamp_ = ss.str();

        try
        {
            run_dir_ = std::filesystem::path(output_dir_) / ("run_" + run_stamp_);
            for (const char* sub : {"maps", "meshes", "normals", "paths", "logs"})
                std::filesystem::create_directories(run_dir_ / sub);

            save_path_         = (run_dir_/"maps"   /("global_map_"       + run_stamp_ + ".pcd" )).string();
            mesh_path_         = (run_dir_/"meshes" /("map_"              + run_stamp_ + ".stl" )).string();
            normals_save_path_ = (run_dir_/"normals"/("selected_normals_" + run_stamp_ + ".yaml")).string();
            path_yaml_         = (run_dir_/"paths"  /("path_"             + run_stamp_ + ".yaml")).string();

            // Repoint <output_dir>/latest atomically: build a temp symlink,
            // then rename over the old one. rename(2) replaces in a single
            // step, so a reader never sees a missing or half-written link.
            const auto link = std::filesystem::path(output_dir_) / "latest";
            const auto tmp  = std::filesystem::path(output_dir_) / ".latest.tmp";
            std::filesystem::remove(tmp);
            std::filesystem::create_directory_symlink(run_dir_, tmp);
            std::filesystem::rename(tmp, link);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to open run directory: %s", e.what());
            return false;
        }

        RCLCPP_INFO(get_logger(), "Run directory: %s", run_dir_.c_str());
        return true;
    }

    void saveMap()
    {
        if (saved_) return;   // avoid duplicate save if destructor also fires after completion

        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!global_map_ || global_map_->empty())
        {
            RCLCPP_WARN(get_logger(), "Global map is empty. Nothing to save.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Saving global map...");
        try
        {
            std::filesystem::path path(save_path_);
            if (path.has_parent_path())
                std::filesystem::create_directories(path.parent_path());

            int result = pcl::io::savePCDFileBinary(save_path_, *global_map_);
            if (result == 0)
            {
                RCLCPP_INFO(get_logger(), "Saved map to %s (%zu points).",
                            save_path_.c_str(), global_map_->size());
                saved_ = true;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Failed to save map to %s.", save_path_.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception while saving map: %s", e.what());
        }
    }

    // -----------------------------------------------
    // MESH SERVICES
    // -----------------------------------------------

    // ---- Load STL, extract per-face centroid + normal, publish RViz marker ----
    // Returns true if mesh was loaded and face geometry is ready for click lookup.
    // The old publishMesh() only built an RViz marker from the file URI.
    // This version also parses the mesh geometry so onClickedPoint can
    // resolve clicks to face normals (wired up in Stage 2).
    bool loadAndPublishMesh(const std::string& stl_path, bool wait_for_file = true)
    {
        // ---- wait for the STL to appear (same poll as before) ----
        const int max_wait_seconds = wait_for_file ? 30 : 0;

        for (int i = 0; i < max_wait_seconds; ++i)
        {
            if (std::filesystem::exists(stl_path))
            {
                break;
            }

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Waiting for mesh file: %s ...", stl_path.c_str());

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        if (!std::filesystem::exists(stl_path))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Mesh file was not created within %d seconds: %s",
                max_wait_seconds,
                stl_path.c_str());
            return false;
        }

        // ---- load the polygon mesh ----
        pcl::PolygonMesh polymesh;
        if (pcl::io::loadPolygonFileSTL(stl_path, polymesh) < 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to parse STL file: %s", stl_path.c_str());
            return false;
        }

        pcl::PointCloud<PointT>::Ptr verts(new pcl::PointCloud<PointT>);
        pcl::fromPCLPointCloud2(polymesh.cloud, *verts);

        if (verts->empty())
        {
            RCLCPP_ERROR(get_logger(), "STL contains no vertices: %s", stl_path.c_str());
            return false;
        }

        // ---- compute per-face centroid + face normal ----
        const auto& polygons = polymesh.polygons;

        mesh_centroids_.reset(new pcl::PointCloud<PointT>);
        mesh_face_normals_.reset(new pcl::PointCloud<pcl::Normal>);
        mesh_centroids_->points.reserve(polygons.size());
        mesh_face_normals_->points.reserve(polygons.size());

        size_t degenerate_count = 0;

        for (const auto& face : polygons)
        {
            if (face.vertices.size() < 3) continue;

            const auto& a = verts->points[face.vertices[0]];
            const auto& b = verts->points[face.vertices[1]];
            const auto& c = verts->points[face.vertices[2]];

            Eigen::Vector3f va(a.x, a.y, a.z);
            Eigen::Vector3f vb(b.x, b.y, b.z);
            Eigen::Vector3f vc(c.x, c.y, c.z);

            // face normal via cross product
            Eigen::Vector3f n = (vb - va).cross(vc - va);
            float len = n.norm();
            if (len < 1e-10f)
            {
                ++degenerate_count;
                continue;   // skip degenerate (zero-area) triangles
            }
            n /= len;

            // centroid
            Eigen::Vector3f cent = (va + vb + vc) / 3.0f;

            PointT cp;
            cp.x = cent.x(); cp.y = cent.y(); cp.z = cent.z();
            mesh_centroids_->points.push_back(cp);

            pcl::Normal fn;
            fn.normal_x = n.x(); fn.normal_y = n.y(); fn.normal_z = n.z();
            mesh_face_normals_->points.push_back(fn);
        }

        mesh_centroids_->width  = mesh_centroids_->points.size();
        mesh_centroids_->height = 1;
        mesh_centroids_->is_dense = true;

        mesh_face_normals_->width  = mesh_face_normals_->points.size();
        mesh_face_normals_->height = 1;
        mesh_face_normals_->is_dense = true;

        if (mesh_centroids_->empty())
        {
            RCLCPP_ERROR(get_logger(), "All mesh faces were degenerate — nothing to click on.");
            return false;
        }

        // KD-tree on centroids for fast click → face lookup
        mesh_tree_.reset(new pcl::search::KdTree<PointT>);
        mesh_tree_->setInputCloud(mesh_centroids_);

        RCLCPP_INFO(
            get_logger(),
            "Loaded mesh: %zu faces (%zu degenerate skipped), centroid KD-tree built.",
            mesh_centroids_->size(), degenerate_count);

        // ---- publish RViz mesh marker (same as old publishMesh) ----
        visualization_msgs::msg::Marker mesh;

        mesh.header.frame_id = frame_id_;
        mesh.header.stamp = get_clock()->now();

        mesh.ns = "reconstructed_mesh";
        mesh.id = 0;

        mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        mesh.action = visualization_msgs::msg::Marker::ADD;

        mesh.mesh_resource = "file://" + stl_path;

        mesh.pose.position.x = 0.0;
        mesh.pose.position.y = 0.0;
        mesh.pose.position.z = 0.0;

        mesh.pose.orientation.x = 0.0;
        mesh.pose.orientation.y = 0.0;
        mesh.pose.orientation.z = 0.0;
        mesh.pose.orientation.w = 1.0;

        mesh.scale.x = 1.0;
        mesh.scale.y = 1.0;
        mesh.scale.z = 1.0;

        mesh.color.r = 0.5f;
        mesh.color.g = 0.5f;
        mesh.color.b = 0.5f;
        mesh.color.a = 0.7;

        mesh_pub_->publish(mesh);

        RCLCPP_INFO(
            get_logger(),
            "Published reconstructed mesh: %s",
            stl_path.c_str());

        return true;
    }

    // Deterministic color from a 3D position — same position always gives same color
    // RANSAC + Cylinder fitting
    void fitAndPublishCylinder(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr& normals,
        const std_msgs::msg::Header& header)
    {
        // RANSAC's sampler indexes with `rnd() % (n - i)`. Feed it a mesh with
        // fewer faces than the model's sample size and the process dies with
        // SIGFPE rather than returning an error. Normals must also match the
        // cloud one-for-one, or the model silently reads past the end.
        if (!cloud || !normals || cloud->size() < 10 ||
            cloud->size() != normals->size())
        {
            RCLCPP_WARN(get_logger(),
                "Skipping cylinder fit: %zu faces, %zu normals.",
                cloud ? cloud->size() : 0, normals ? normals->size() : 0);
            has_cylinder_ = false;
            return;
        }
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(max_ransac_iterations_);         // TUNE: max RANSAC iterations
        seg.setDistanceThreshold(0.01);          // max distance (m) a point can be from the fitted surface
        seg.setProbability(ransac_probability_);                // RANSAC stops early once it's statistically confident. 
        seg.setRadiusLimits(min_ransac_radius_, max_ransac_radius_);        // TUNE: expected tube radius range in metres

        seg.setInputCloud(cloud);
        seg.setInputNormals(normals);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(get_logger(), "No cylinder model found");
            has_cylinder_ = false;
            return;
        }

        // Cylinder coefficients: [point_on_axis(x,y,z), axis_direction(x,y,z), radius]
        Eigen::Vector3f axis_point(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Vector3f axis_dir(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
        axis_dir.normalize();
        float radius = coefficients->values[6];

        RCLCPP_INFO(get_logger(), "Cylinder fit: %zu inliers, radius=%.3f m, axis=(%.2f,%.2f,%.2f)",
                    inliers->indices.size(), radius, axis_dir.x(), axis_dir.y(), axis_dir.z());

        // Project each inlier onto the axis to find the cylinder's extent (where it starts/ends)
        float min_t = std::numeric_limits<float>::max();
        float max_t = std::numeric_limits<float>::lowest();
        for (int idx : inliers->indices) {
            Eigen::Vector3f p = cloud->points[idx].getVector3fMap();
            float t = (p - axis_point).dot(axis_dir);
            min_t = std::min(min_t, t);
            max_t = std::max(max_t, t);
        }

        // Points within this band of either end = candidate graspable rim
        float rim_band = 0.02f;  // TUNE: metres from each end counted as "rim"

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rim_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (int idx : inliers->indices) {
            Eigen::Vector3f p = cloud->points[idx].getVector3fMap();
            float t = (p - axis_point).dot(axis_dir);
            bool near_min = (t - min_t) < rim_band;
            bool near_max = (max_t - t) < rim_band;
            if (near_min || near_max) {
            pcl::PointXYZRGB pt;
            pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
            if (near_min) { pt.r = 255; pt.g = 0;   pt.b = 0;   }  // one end = red
            else          { pt.r = 0;   pt.g = 0;   pt.b = 255; }  // other end = blue
            rim_cloud->points.push_back(pt);
            }
        }
        rim_cloud->width = rim_cloud->points.size();
        rim_cloud->height = 1;

        sensor_msgs::msg::PointCloud2 rim_msg;
        pcl::toROSMsg(*rim_cloud, rim_msg);
        rim_msg.header = header;
        rim_pub_->publish(rim_msg);

        // Visualize the fitted axis as a line
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker axis_marker;
        axis_marker.header = header;
        axis_marker.ns = "cylinder_axis";
        axis_marker.id = 0;
        axis_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        axis_marker.action = visualization_msgs::msg::Marker::ADD;
        axis_marker.scale.x = 0.005;
        axis_marker.color.r = 1.0f; axis_marker.color.g = 1.0f; axis_marker.color.b = 0.0f; axis_marker.color.a = 1.0f;
        axis_marker.lifetime = rclcpp::Duration::from_seconds(0);

        Eigen::Vector3f p_start = axis_point + axis_dir * min_t;
        Eigen::Vector3f p_end   = axis_point + axis_dir * max_t;
        geometry_msgs::msg::Point gs, ge;
        gs.x = p_start.x(); gs.y = p_start.y(); gs.z = p_start.z();
        ge.x = p_end.x();   ge.y = p_end.y();   ge.z = p_end.z();
        axis_marker.points.push_back(gs);
        axis_marker.points.push_back(ge);

        marker_array.markers.push_back(axis_marker);
        cylinder_marker_pub_->publish(marker_array);

        // Store for later use (e.g. picking a rim point as a grasp target)
        last_cylinder_mid_        = 0.5f * (p_start + p_end);   // centroid of the drawn axis
        last_cylinder_axis_point_ = axis_point;
        last_cylinder_axis_dir_   = axis_dir;
        last_cylinder_radius_     = radius;
        has_cylinder_             = true;
    }
    // -----------------------------------------------
    // NORMALS SERVICES
    // -----------------------------------------------

    void onClickedPoint(
        const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // ---------------------------------------------------------
        // 0a. Cluster removal takes the click first. While armed, a
        //     click picks a cluster instead of a normal — one RViz tool
        //     (Publish Point), two meanings, chosen by mode.
        // ---------------------------------------------------------
        if (cluster_armed_)
        {
            onClusterClick(msg);
            return;
        }

                // ---------------------------------------------------------
        // 0. Clicks are allowed as soon as any mesh is loaded —
        //    live during collection, or the final mesh after it.
        // ---------------------------------------------------------
        if (!pipeline_done_ && loaded_mesh_path_.empty())
        {
            RCLCPP_WARN(
                get_logger(),
                "No mesh loaded yet — wait for the first live mesh.");
            return;
        }

        // ---------------------------------------------------------
        // 1. Make sure mesh face data is available
        // ---------------------------------------------------------
        if (!mesh_centroids_ || !mesh_tree_ || !mesh_face_normals_)
        {
            RCLCPP_WARN(
                get_logger(),
                "Mesh face data not loaded yet, ignoring click");
            return;
        }

        // ---------------------------------------------------------
        // 2. Find nearest mesh face centroid to RViz click (KD-tree)
        // ---------------------------------------------------------
        PointT click_pt;
        click_pt.x = static_cast<float>(msg->point.x);
        click_pt.y = static_cast<float>(msg->point.y);
        click_pt.z = static_cast<float>(msg->point.z);

        std::vector<int>   idx(1);
        std::vector<float> dist2(1);
        if (mesh_tree_->nearestKSearch(click_pt, 1, idx, dist2) < 1)
        {
            RCLCPP_WARN(get_logger(), "KD-tree search returned no result");
            return;
        }

        int best_idx = idx[0];
        float best_dist2 = dist2[0];

        // ---------------------------------------------------------
        // 3. Get face centroid + face normal
        // ---------------------------------------------------------
        const auto& clicked_point =
            mesh_centroids_->points[best_idx];

        const auto& normal =
            mesh_face_normals_->points[best_idx];

        Eigen::Vector3f normal_vec(
            normal.normal_x, normal.normal_y, normal.normal_z);

        if (normal_vec.norm() < 1e-6f)
        {
            RCLCPP_WARN(
                get_logger(),
                "Nearest face has zero-length normal, ignoring click");
            return;
        }

        normal_vec.normalize();

        // ---------------------------------------------------------
        // 4. Print selection information
        // ---------------------------------------------------------
        RCLCPP_INFO(
            get_logger(),
            "Clicked face centroid: (%.3f, %.3f, %.3f), "
            "face normal: (%.3f, %.3f, %.3f), distance to click: %.4f m",
            clicked_point.x,
            clicked_point.y,
            clicked_point.z,
            normal_vec.x(),
            normal_vec.y(),
            normal_vec.z(),
            std::sqrt(best_dist2));

        // ---------------------------------------------------------
        // 6. Check whether this point is already selected
        // ---------------------------------------------------------
        int hit = -1;

        const double merge_radius2 =
            normal_merge_radius_ *
            normal_merge_radius_;

        for (size_t i = 0;
            i < selected_normals_.size();
            ++i)
        {
            const auto& q =
                selected_normals_[i].pose.position;

            double dx = q.x - clicked_point.x;
            double dy = q.y - clicked_point.y;
            double dz = q.z - clicked_point.z;

            double d2 =
                dx * dx +
                dy * dy +
                dz * dz;

            if (d2 < merge_radius2)
            {
                hit = static_cast<int>(i);
                break;
            }
        }

        // ---------------------------------------------------------
        // 7. Toggle selection
        // ---------------------------------------------------------
        if (hit >= 0)
        {
            // Existing normal -> remove it
            selected_normals_.erase(
                selected_normals_.begin() + hit);

            RCLCPP_INFO(
                get_logger(),
                "Removed normal #%d (%zu left)",
                hit,
                selected_normals_.size());
        }
        else
        {
            // New normal -> add it
            geometry_msgs::msg::PoseStamped target;

            target.header.frame_id = frame_id_;
            target.header.stamp = now();

            target.pose.position.x =
                clicked_point.x;

            target.pose.position.y =
                clicked_point.y;

            target.pose.position.z =
                clicked_point.z;

            // Orient local +X along surface normal
            Eigen::Quaternionf qr =
                Eigen::Quaternionf::FromTwoVectors(
                    Eigen::Vector3f(1.0f, 0.0f, 0.0f),
                    normal_vec);

            target.pose.orientation.x = qr.x();
            target.pose.orientation.y = qr.y();
            target.pose.orientation.z = qr.z();
            target.pose.orientation.w = qr.w();

            selected_normals_.push_back(target);

            RCLCPP_INFO(
                get_logger(),
                "Added normal #%zu at "
                "(%.3f, %.3f, %.3f)",
                selected_normals_.size() - 1,
                clicked_point.x,
                clicked_point.y,
                clicked_point.z);
        }

        // ---------------------------------------------------------
        // 8. Publish + save all selected normals
        // ---------------------------------------------------------
        publishSelectedNormals();
        //saveSelectedNormals(); // don't save yet, waiting for user to finish selection. 
    }

    // Saves the current selection to normals/selected_normals_<stamp>.yaml,
    // overwriting the previous save. Selection stays open.
    void saveNormalsService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (normals_save_path_.empty()) {
            response->success = false;
            response->message = "No run started yet.";
            return;
        }

        if (selected_normals_.empty()) {
            response->success = false;
            response->message = "No normals have been selected.";
            return;
        }

        if (!saveSelectedNormals()) {
            response->success = false;
            response->message = "Failed to save selected normals.";
            return;
        }

        RCLCPP_INFO(get_logger(), "Saved %zu normal(s) to %s",
                    selected_normals_.size(), normals_save_path_.c_str());

        response->success = true;
        response->message = normals_save_path_;   // shell uses this as the file path
    }

    void undoNormalSelection(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!pipeline_done_ && loaded_mesh_path_.empty())
        {
            response->success = false;
            response->message =
                "No mesh loaded yet.";
            return;
        }

        if (selected_normals_.empty())
        {
            response->success = false;
            response->message =
                "Nothing to undo.";
            return;
        }

        // Remove the most recently selected normal
        selected_normals_.pop_back();

        // Redraw the remaining normals
        publishSelectedNormals();

        RCLCPP_INFO(
            get_logger(),
            "Undid most recently selected normal. "
            "%zu normal(s) remaining.",
            selected_normals_.size());

        response->success = true;
        response->message =
            "Removed the most recently selected normal.";
    }

    void clearNormals(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!pipeline_done_ && loaded_mesh_path_.empty())
        {
            response->success = false;
            response->message = "No mesh loaded yet.";
            return;
        }

        if (selected_normals_.empty())
        {
            response->success = false;
            response->message = "No normals to clear.";
            return;
        }

        const size_t n = selected_normals_.size();
        selected_normals_.clear();

        // Publishes DELETEALL + an empty PoseArray, so RViz is wiped too.
        publishSelectedNormals();

        RCLCPP_INFO(get_logger(), "Cleared all %zu selected normal(s).", n);

        response->success = true;
        response->message = "Cleared " + std::to_string(n) + " normal(s).";
    }

    void publishSelectedNormals()
    {
        auto stamp = get_clock()->now();

        geometry_msgs::msg::PoseArray pa;
        pa.header.frame_id = frame_id_;
        pa.header.stamp = stamp;
        for (const auto& t : selected_normals_) pa.poses.push_back(t.pose);
        targets_pub_->publish(pa);

        visualization_msgs::msg::MarkerArray ma;
        visualization_msgs::msg::Marker wipe;      // clear removed ones
        wipe.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(wipe);

        int id = 0;
        for (size_t i = 0; i < selected_normals_.size(); ++i) {
            const auto& p = selected_normals_[i].pose.position;
            const auto& o = selected_normals_[i].pose.orientation;
            Eigen::Vector3f n = Eigen::Quaternionf(o.w, o.x, o.y, o.z) * Eigen::Vector3f(1,0,0);

            visualization_msgs::msg::Marker a;
            a.header.frame_id = frame_id_;
            a.header.stamp = stamp;
            a.ns = "selected_normals";
            a.id = id++;
            a.type = visualization_msgs::msg::Marker::ARROW;
            a.action = visualization_msgs::msg::Marker::ADD;
            geometry_msgs::msg::Point p0, p1;
            p0.x = p.x; p0.y = p.y; p0.z = p.z;
            p1.x = p.x + n.x()*0.08; p1.y = p.y + n.y()*0.08; p1.z = p.z + n.z()*0.08;
            a.points = {p0, p1};
            a.scale.x = 0.004; a.scale.y = 0.008; a.scale.z = 0.012;
            a.color.g = 1.0f; a.color.a = 1.0f;
            a.pose.orientation.w = 1.0;
            ma.markers.push_back(a);

            visualization_msgs::msg::Marker txt;
            txt.header = a.header;
            txt.ns = "selected_normals_label";
            txt.id = id++;
            txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::msg::Marker::ADD;
            txt.pose.position.x = p.x; txt.pose.position.y = p.y; txt.pose.position.z = p.z + 0.03;
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 0.02;
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0f;
            txt.text = std::to_string(i);
            ma.markers.push_back(txt);
        }
        targets_markers_pub_->publish(ma);
    }

    bool saveSelectedNormals(const std::string& override_path = "")
    {
        const std::string out = override_path.empty() ? normals_save_path_ : override_path;
        if (selected_normals_.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "No selected normals to save.");
            return false;
        }

        try {
            YAML::Node root;

            root["frame_id"] =
                selected_normals_.front().header.frame_id;

            root["count"] =
                static_cast<int>(selected_normals_.size());

            YAML::Node targets(YAML::NodeType::Sequence);

            for (const auto &target : selected_normals_) {

                YAML::Node node;

                node["position"]["x"] =
                    target.pose.position.x;
                node["position"]["y"] =
                    target.pose.position.y;
                node["position"]["z"] =
                    target.pose.position.z;

                node["orientation"]["x"] =
                    target.pose.orientation.x;
                node["orientation"]["y"] =
                    target.pose.orientation.y;
                node["orientation"]["z"] =
                    target.pose.orientation.z;
                node["orientation"]["w"] =
                    target.pose.orientation.w;

                targets.push_back(node);
            }

            root["targets"] = targets;

            std::ofstream fout(out);

            if (!fout.is_open()) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Could not open normals file: %s",
                    out.c_str());

                return false;
            }

            fout << root;
            fout.close();

            RCLCPP_INFO(
                this->get_logger(),
                "Saved %zu selected normals to %s",
                selected_normals_.size(),
                out.c_str());

            return true;
        }
        catch (const YAML::Exception &e) {

            RCLCPP_ERROR(
                this->get_logger(),
                "YAML error while saving normals: %s",
                e.what());

            return false;
        }
        catch (const std::exception &e) {

            RCLCPP_ERROR(
                this->get_logger(),
                "Error while saving normals: %s",
                e.what());

            return false;
        }
        }


    bool loadSelectedNormals()
    {
        if (!std::filesystem::exists(normals_save_path_)) return false;
        YAML::Node root = YAML::LoadFile(normals_save_path_);
        selected_normals_.clear();
        for (const auto& n : root["targets"]) {
            geometry_msgs::msg::PoseStamped t;
            t.header.frame_id = root["frame_id"].as<std::string>(frame_id_);
            t.pose.position.x = n["position"]["x"].as<double>();
            t.pose.position.y = n["position"]["y"].as<double>();
            t.pose.position.z = n["position"]["z"].as<double>();
            t.pose.orientation.x = n["orientation"]["x"].as<double>();
            t.pose.orientation.y = n["orientation"]["y"].as<double>();
            t.pose.orientation.z = n["orientation"]["z"].as<double>();
            t.pose.orientation.w = n["orientation"]["w"].as<double>();
            selected_normals_.push_back(t);
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu normals", selected_normals_.size());
        return !selected_normals_.empty();
    }

    // Dump the current selection to normals/live_NNNNN.yaml without ending
    // selection. Returns the path so the caller can feed it to the spline.
    void dumpNormals(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (selected_normals_.size() < 2)
        {
            res->success = false;
            res->message = "Need at least 2 normals for a path (have "
                         + std::to_string(selected_normals_.size()) + ").";
            return;
        }

        char buf[32];
        std::snprintf(buf, sizeof(buf), "live_%05d.yaml", snapshot_count_);
        const auto out = (run_dir_ / "normals" / buf).string();

        if (!saveSelectedNormals(out))
        {
            res->success = false;
            res->message = "Failed to write " + out;
            return;
        }

        res->success = true;
        res->message = out;
    }

    // Parses the locked target file into `out`. `out` is only written on
    // success, so a missing or bad file can never wipe anything.
    bool loadLockedTargets(std::vector<geometry_msgs::msg::PoseStamped>& out,
                           std::string& error)
    {
        if (locked_targets_path_.empty()) {
            error = "locked_target_path is empty.";
            return false;
        }
        if (!std::filesystem::exists(locked_targets_path_)) {
            error = "Locked target file does not exist: " + locked_targets_path_;
            return false;
        }

        try
        {
            YAML::Node root = YAML::LoadFile(locked_targets_path_);

            if (!root["targets"] || !root["targets"].IsSequence()) {
                error = "Invalid locked target YAML: targets missing or not a sequence.";
                return false;
            }

            const std::string frame = root["frame_id"]
                ? root["frame_id"].as<std::string>() : frame_id_;
            if (frame != frame_id_) {
                RCLCPP_WARN(get_logger(),
                    "Locked targets are in frame '%s', pipeline uses '%s'.",
                    frame.c_str(), frame_id_.c_str());
            }

            std::vector<geometry_msgs::msg::PoseStamped> parsed;
            for (const auto& node : root["targets"])
            {
                geometry_msgs::msg::PoseStamped t;
                t.header.frame_id = frame;
                t.header.stamp = get_clock()->now();
                t.pose.position.x    = node["position"]["x"].as<double>();
                t.pose.position.y    = node["position"]["y"].as<double>();
                t.pose.position.z    = node["position"]["z"].as<double>();
                t.pose.orientation.x = node["orientation"]["x"].as<double>();
                t.pose.orientation.y = node["orientation"]["y"].as<double>();
                t.pose.orientation.z = node["orientation"]["z"].as<double>();
                t.pose.orientation.w = node["orientation"]["w"].as<double>();
                parsed.push_back(t);
            }

            if (parsed.empty()) {
                error = "Locked target file has no targets.";
                return false;
            }

            out = std::move(parsed);
            return true;
        }
        catch (const YAML::Exception& e)
        {
            error = std::string("Failed to parse locked target file: ") + e.what();
            return false;
        }
    }

    void loadLockedTargetService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!pipeline_done_ && loaded_mesh_path_.empty()) {
            response->success = false;
            response->message = "No mesh loaded yet.";
            return;
        }

        std::vector<geometry_msgs::msg::PoseStamped> loaded;
        std::string error;
        if (!loadLockedTargets(loaded, error)) {
            response->success = false;
            response->message = error;
            return;
        }

        size_t added = 0, skipped = 0;
        for (const auto& t : loaded)
        {
            const auto& p = t.pose.position;
            const bool dup = std::any_of(
                selected_normals_.begin(), selected_normals_.end(),
                [&](const geometry_msgs::msg::PoseStamped& s) {
                    const double dx = p.x - s.pose.position.x;
                    const double dy = p.y - s.pose.position.y;
                    const double dz = p.z - s.pose.position.z;
                    return dx*dx + dy*dy + dz*dz < 1e-6;   // within 1 mm
                });
            if (dup) { ++skipped; continue; }
            selected_normals_.push_back(t);
            ++added;
        }

        publishSelectedNormals();

        RCLCPP_INFO(get_logger(),
            "Locked targets: added %zu, skipped %zu duplicate(s), %zu total.",
            added, skipped, selected_normals_.size());

        response->success = added > 0;
        response->message =
            "Added " + std::to_string(added) + " locked normal(s), skipped " +
            std::to_string(skipped) + " already selected. Total " +
            std::to_string(selected_normals_.size()) + ".";
    }

    void saveLockedTarget()
    {
        if (!has_locked_target_)
        {
            RCLCPP_WARN(get_logger(), "No locked target to save.");
            return;
        }

        std::ofstream file(locked_targets_path_);

        file << std::fixed << std::setprecision(6);

        file << "frame_id: " << locked_target_.header.frame_id << "\n";

        file << "position:\n";
        file << "  x: " << locked_target_.pose.position.x << "\n";
        file << "  y: " << locked_target_.pose.position.y << "\n";
        file << "  z: " << locked_target_.pose.position.z << "\n";

        file << "orientation:\n";
        file << "  x: " << locked_target_.pose.orientation.x << "\n";
        file << "  y: " << locked_target_.pose.orientation.y << "\n";
        file << "  z: " << locked_target_.pose.orientation.z << "\n";
        file << "  w: " << locked_target_.pose.orientation.w << "\n";

        file.close();

        RCLCPP_INFO(
            get_logger(),
            "Saved locked target to %s", locked_targets_path_.c_str());
    }

    // -----------------------------------------------
    // PATH SERVICES
    // -----------------------------------------------

    void loadAndPublishPathService(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        if (!generate_path_)
        {
            response->success = false;
            response->message = "Path generation is disabled.";
            return;
        }

        std::string yaml = path_yaml_;
        if (collecting_)
        {
            std::string newest;
            try {
                for (const auto& e : std::filesystem::directory_iterator(run_dir_/"paths"))
                {
                    const auto n = e.path().filename().string();
                    if (n.rfind("live_", 0) == 0 && e.path().extension() == ".yaml")
                        if (newest.empty() || n > std::filesystem::path(newest).filename().string())
                            newest = e.path().string();
                }
            } catch (const std::exception&) {}
            if (!newest.empty()) yaml = newest;
        }

        if (yaml.empty())
        {
            response->success = false;
            response->message = "No path YAML available.";
            return;
        }

        if (!std::filesystem::exists(yaml))
        {
            response->success = false;
            response->message =
                "Path YAML does not exist: " + yaml;
            return;
        }

        bool success = loadAndPublishPath(yaml);

        response->success = success;

        if (success)
        {
            response->message =
                "Path loaded and published: " + yaml;
        }
        else
        {
            response->message =
                "Failed to load and publish path.";
        }
    }

    bool loadAndPublishPath(const std::string& path_yaml)
    {
        if (path_yaml.empty())
        {
            RCLCPP_WARN(
                get_logger(),
                "Path YAML path is empty.");
            return false;
        }

        if (!std::filesystem::exists(path_yaml))
        {
            RCLCPP_WARN(
                get_logger(),
                "Path YAML does not exist: %s",
                path_yaml.c_str());
            return false;
        }

        try
        {
            YAML::Node root = YAML::LoadFile(path_yaml);

            if (!root["waypoints"] ||
                !root["waypoints"].IsSequence())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Invalid path YAML: 'waypoints' missing or not a sequence.");

                return false;
            }

            std::string frame =
                root["frame_id"]
                    ? root["frame_id"].as<std::string>()
                    : frame_id_;

            const auto& waypoints = root["waypoints"];

            if (waypoints.size() < 2)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Path contains only %zu waypoint(s). "
                    "At least 2 are required.",
                    waypoints.size());

                return false;
            }

            nav_msgs::msg::Path path_msg;

            path_msg.header.frame_id = frame;
            path_msg.header.stamp = now();

            for (std::size_t i = 0; i < waypoints.size(); ++i)
            {
                const auto& wp = waypoints[i];

                if (!wp["position"] || !wp["orientation"])
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Skipping waypoint %zu: missing position or orientation.",
                        i);
                    continue;
                }

                geometry_msgs::msg::PoseStamped pose;

                pose.header = path_msg.header;

                // Position
                pose.pose.position.x =
                    wp["position"]["x"].as<double>();

                pose.pose.position.y =
                    wp["position"]["y"].as<double>();

                pose.pose.position.z =
                    wp["position"]["z"].as<double>();

                // Orientation
                pose.pose.orientation.x =
                    wp["orientation"]["x"].as<double>();

                pose.pose.orientation.y =
                    wp["orientation"]["y"].as<double>();

                pose.pose.orientation.z =
                    wp["orientation"]["z"].as<double>();

                pose.pose.orientation.w =
                    wp["orientation"]["w"].as<double>();

                path_msg.poses.push_back(pose);
            }

            if (path_msg.poses.empty())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "No valid waypoints found in path YAML.");

                return false;
            }

            // ---------------------------------------------------------
            // Publish
            // ---------------------------------------------------------

            path_pub_->publish(path_msg);

            RCLCPP_INFO(
                get_logger(),
                "Published path: %zu waypoints, frame: %s",
                path_msg.poses.size(),
                frame.c_str());

            return true;
            }
            catch (const YAML::Exception& e)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to load path YAML: %s",
                    e.what());

                return false;
            }
    }

    void clearPathService(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        // An empty Path replaces the displayed one in RViz, and also
        // replaces the transient_local copy late subscribers receive.
        nav_msgs::msg::Path empty;
        empty.header.frame_id = frame_id_;   // non-empty, or RViz warns about TF
        empty.header.stamp = now();
        path_pub_->publish(empty);

        RCLCPP_INFO(get_logger(), "Cleared published path.");

        response->success = true;
        response->message = "Path cleared (files on disk kept).";
    }


    // Clear all RViz visualizations (point clouds, markers, mesh) before starting a new collection or after saving the map

    // =========================================================
    // CLUSTER REMOVAL
    //
    // Purpose: a scan of a tree trunk should end up as ONE connected
    // cluster. Anything else in the box — a pot, a wall, a bench leg —
    // is a separate cluster, and deleting it here is cheaper than
    // re-scanning with a tighter crop box.
    //
    // Flow:
    //   /cluster_map              cluster the map, refuse if <= 1 cluster
    //   click in RViz             pick the cluster under the click
    //   /confirm_cluster_removal  delete it from global_map_
    //   /cancel_cluster_removal   drop the selection, change nothing
    //
    // Nothing is deleted until the confirm service is called, so the GUI
    // can put a dialog between the click and the deletion.
    // =========================================================

    // One line of machine-readable state for the GUI, latched so a panel
    // that connects late still sees it. Pipe-separated because the panel
    // only has to split() it — no new message package to build.
    //   idle
    //   armed|<n_clusters>
    //   pending|<index>|<n_clusters>|<points_in_cluster>|<x>|<y>|<z>
    void publishClusterStatus(const std::string& line)
    {
        std_msgs::msg::String msg;
        msg.data = line;
        cluster_status_pub_->publish(msg);
    }

    // Drop the armed selection and repaint. Called from the cancel
    // service, after a successful removal, and whenever the map changes
    // underneath us (/clear_map, /start_collection) — cluster_indices_
    // index into cluster_cloud_, so a stale pair could delete the wrong
    // points entirely.
    void resetClusterSelection()
    {
        cluster_armed_    = false;
        pending_cluster_  = -1;
        cluster_cloud_.reset();
        cluster_tree_.reset();
        cluster_indices_.clear();
        cluster_label_.clear();
        publishClusterPreview();     // empty cloud: RViz clears the overlay
        publishClusterStatus("idle");
    }

    // Publish the clustered copy as XYZRGB: one colour per cluster, red
    // for the pending one, dark grey for points no cluster claimed.
    // Empty cloud when nothing is armed — toROSMsg still writes the x/y/z
    // fields, which is what makes RViz drop the old overlay rather than
    // keep showing it.
    void publishClusterPreview()
    {
        static const uint8_t palette[10][3] = {
            { 46, 204, 113}, { 52, 152, 219}, {241, 196,  15},
            {155,  89, 182}, { 26, 188, 156}, {230, 126,  34},
            { 52,  73,  94}, { 39, 174,  96}, {142,  68, 173},
            {127, 140, 141},
        };

        pcl::PointCloud<pcl::PointXYZRGB> rgb;

        if (cluster_cloud_ && cluster_label_.size() == cluster_cloud_->size())
        {
            rgb.reserve(cluster_cloud_->size());
            for (std::size_t i = 0; i < cluster_cloud_->size(); ++i)
            {
                const auto& p = cluster_cloud_->points[i];
                pcl::PointXYZRGB q;
                q.x = p.x; q.y = p.y; q.z = p.z;

                const int label = cluster_label_[i];
                if (label < 0)                        // below min_points
                {
                    q.r = q.g = q.b = 80;
                }
                else if (label == pending_cluster_)   // about to be deleted
                {
                    q.r = 255; q.g = 40; q.b = 40;
                }
                else
                {
                    const uint8_t* c = palette[label % 10];
                    q.r = c[0]; q.g = c[1]; q.b = c[2];
                }
                rgb.push_back(q);
            }
        }

        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(rgb, out);
        out.header.frame_id = frame_id_;
        out.header.stamp = now();
        clusters_pub_->publish(out);
    }

    // ---- /cluster_map : split the map and arm a selection ----
    void clusterMapService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // Same guard as /clear_map: finalizeRun() is queued on a 1 ms timer
        // and is about to read global_map_ for the final save and mesh.
        if (finalize_timer_ && !finalize_timer_->is_canceled())
        {
            response->success = false;
            response->message = "Run is finalising. Try again once the final mesh is loaded.";
            return;
        }

        // Work on a voxelised COPY, taken under the lock and released
        // immediately: cloudCallback() keeps accumulating while we cluster.
        pcl::PointCloud<pcl::PointXYZ>::Ptr work(new pcl::PointCloud<pcl::PointXYZ>);
        std::size_t map_points = 0;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (!global_map_ || global_map_->size() < 100)
            {
                response->success = false;
                response->message = "Map is empty or too sparse to cluster.";
                return;
            }
            map_points = global_map_->size();

            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(global_map_);
            vg.setLeafSize(cluster_voxel_, cluster_voxel_, cluster_voxel_);
            vg.filter(*work);
        }

        // EuclideanClusterExtraction is a flood fill over a radius graph:
        // seed a point, absorb everything within cluster_tolerance_, repeat
        // from each absorbed point until nothing new joins. The KD-tree is
        // what makes each neighbour query log-time instead of linear.
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(work);

        std::vector<pcl::PointIndices> indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(cluster_min_points_);
        ec.setMaxClusterSize(cluster_max_points_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(work);

        t.start();
        ec.extract(indices);
        const double cluster_ms = t.stop_ms();

        // extract() already returns clusters largest-first, but that is
        // documented behaviour rather than a guarantee, and the indices
        // are what the GUI shows the user — so sort explicitly.
        std::sort(indices.begin(), indices.end(),
                  [](const pcl::PointIndices& a, const pcl::PointIndices& b)
                  { return a.indices.size() > b.indices.size(); });

        RCLCPP_INFO(get_logger(),
            "cluster_map: %zu points -> %zu voxels -> %zu cluster(s) (%.1f ms)",
            map_points, work->size(), indices.size(), cluster_ms);

        if (indices.size() <= 1)
        {
            // Exactly what a clean trunk scan should look like: refuse,
            // and leave the map untouched.
            resetClusterSelection();
            response->success = false;
            response->message =
                indices.empty()
                ? "No cluster reached cluster_min_points — nothing to remove."
                : "Only 1 cluster — the scan is already a single object.";
            return;
        }

        // Per-point label, so a click resolves to a cluster with one
        // nearest-neighbour lookup instead of a scan over every cluster.
        cluster_label_.assign(work->size(), -1);
        for (std::size_t c = 0; c < indices.size(); ++c)
            for (int i : indices[c].indices)
                cluster_label_[static_cast<std::size_t>(i)] = static_cast<int>(c);

        cluster_cloud_   = work;
        cluster_tree_    = tree;
        cluster_indices_ = indices;
        pending_cluster_ = -1;
        cluster_armed_   = true;

        publishClusterPreview();
        publishClusterStatus("armed|" + std::to_string(indices.size()));

        std::ostringstream msg;
        msg << indices.size() << " clusters (";
        for (std::size_t c = 0; c < indices.size() && c < 5; ++c)
            msg << (c ? ", " : "") << indices[c].indices.size();
        if (indices.size() > 5) msg << ", ...";
        msg << " voxels). Click a point in the cluster to remove.";

        response->success = true;
        response->message = msg.str();
    }

    // ---- a click while armed : pick the cluster under it ----
    void onClusterClick(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!cluster_cloud_ || !cluster_tree_)
        {
            RCLCPP_WARN(get_logger(), "Cluster click with no clustered cloud.");
            return;
        }

        PointT click_pt;
        click_pt.x = static_cast<float>(msg->point.x);
        click_pt.y = static_cast<float>(msg->point.y);
        click_pt.z = static_cast<float>(msg->point.z);

        std::vector<int>   idx(1);
        std::vector<float> dist2(1);
        if (cluster_tree_->nearestKSearch(click_pt, 1, idx, dist2) < 1)
        {
            RCLCPP_WARN(get_logger(), "Cluster KD-tree returned no result.");
            return;
        }

        const double dist = std::sqrt(static_cast<double>(dist2[0]));
        if (dist > cluster_click_radius_)
        {
            RCLCPP_WARN(get_logger(),
                "Click was %.3f m from the nearest clustered point "
                "(limit %.3f m) — ignored.", dist, cluster_click_radius_);
            return;
        }

        const int label = cluster_label_[static_cast<std::size_t>(idx[0])];
        if (label < 0)
        {
            RCLCPP_WARN(get_logger(),
                "That point belongs to no cluster (below cluster_min_points).");
            return;
        }

        pending_cluster_ = label;
        publishClusterPreview();     // repaint: the pick turns red

        std::ostringstream line;
        line << "pending|" << label
             << "|" << cluster_indices_.size()
             << "|" << cluster_indices_[label].indices.size()
             << "|" << msg->point.x
             << "|" << msg->point.y
             << "|" << msg->point.z;
        publishClusterStatus(line.str());

        RCLCPP_INFO(get_logger(),
            "Cluster %d of %zu selected (%zu voxels), click %.3f m away. "
            "Waiting for confirmation.",
            label, cluster_indices_.size(),
            cluster_indices_[label].indices.size(), dist);
    }

    // ---- /confirm_cluster_removal : delete the pending cluster ----
    void confirmClusterRemovalService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!cluster_armed_ || pending_cluster_ < 0)
        {
            response->success = false;
            response->message = "No cluster selected — click one in RViz first.";
            return;
        }

        const auto& picked = cluster_indices_[pending_cluster_].indices;

        // The cluster is made of VOXEL CENTRES, not of real map points, so
        // it cannot be erased by index. Instead: build a KD-tree on the
        // real map, and drop every real point within one voxel of a picked
        // centre. A voxel's half-diagonal is 0.866 x leaf, so a radius of
        // one leaf covers the whole cell with margin.
        const double radius = std::max(cluster_voxel_, 1e-4);

        std::size_t before = 0, after = 0;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (!global_map_ || global_map_->empty())
            {
                response->success = false;
                response->message = "Map is empty — nothing to remove.";
                return;
            }
            before = global_map_->size();

            pcl::KdTreeFLANN<pcl::PointXYZ> tree;
            tree.setInputCloud(global_map_);

            std::vector<char> drop(before, 0);     // vector<bool> is a bitfield; char is plain
            std::vector<int>   nb;
            std::vector<float> nd;

            t.start();
            for (int i : picked)
            {
                if (tree.radiusSearch(cluster_cloud_->points[i], radius, nb, nd) > 0)
                    for (int j : nb)
                        drop[static_cast<std::size_t>(j)] = 1;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr kept(new pcl::PointCloud<pcl::PointXYZ>);
            kept->reserve(before);
            for (std::size_t k = 0; k < before; ++k)
                if (!drop[k]) kept->push_back(global_map_->points[k]);

            // push_back maintains width/height for an unorganised cloud,
            // but set them explicitly: an inconsistent header makes
            // savePCDFileBinary write a file PCL then refuses to read.
            kept->width    = static_cast<uint32_t>(kept->size());
            kept->height   = 1;
            kept->is_dense = global_map_->is_dense;

            global_map_ = kept;
            after = kept->size();
        }

        const double remove_ms = t.stop_ms();
        const std::size_t removed = before - after;

        resetClusterSelection();

        // Republish so RViz shows the map without the cluster immediately.
        {
            sensor_msgs::msg::PointCloud2 out;
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                pcl::toROSMsg(*global_map_, out);
            }
            out.header.frame_id = frame_id_;
            out.header.stamp = now();
            global_map_pub_->publish(out);

            // Only the finished run publishes /processed/map, and the
            // keepalive timer republishes last_processed_msg_ every second
            // — so that cached copy has to be updated too, or the old map
            // comes straight back.
            if (pipeline_done_)
            {
                processed_pub_->publish(out);
                last_processed_msg_ = out;
            }
        }

        // Make the change reach the mesh, which is what the click-to-select
        // and cylinder fit actually use.
        std::string note;
        if (collecting_)
        {
            // Snapshot now rather than waiting up to snapshot_period_sec_;
            // the mesh watcher picks it up and the live mesh catches up.
            writeSnapshot();
            note = " New snapshot written; the live mesh will catch up.";
        }
        else
        {
            // Overwrite this run's PCD so a re-mesh uses the cleaned cloud.
            saved_ = false;
            saveMap();
            note = " Saved map rewritten: " + save_path_
                 + " — rebuild the mesh to update it.";
        }

        std::ostringstream msg;
        msg << "Removed " << removed << " of " << before << " points ("
            << after << " left, " << std::fixed << std::setprecision(1)
            << remove_ms << " ms)." << note;

        RCLCPP_INFO(get_logger(), "confirm_cluster_removal: %s", msg.str().c_str());
        response->success = true;
        response->message = msg.str();
    }

    // ---- /cancel_cluster_removal : drop the selection, change nothing ----
    void cancelClusterRemovalService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        const bool was_armed = cluster_armed_;
        resetClusterSelection();

        RCLCPP_INFO(get_logger(), "cancel_cluster_removal: selection dropped.");
        response->success = true;
        response->message = was_armed ? "Cluster selection cancelled."
                                      : "No cluster selection was active.";
    }

     void clearRVizVisualizations()
    {
        RCLCPP_INFO(
            get_logger(),
            "Clearing previous RViz visualisations...");

        // ---------------------------------------------------------
        // Clear global map
        // ---------------------------------------------------------

        // RViz drops any PointCloud2 without x/y/z fields and keeps
        // showing the old cloud, so the empty message must still declare
        // them. toROSMsg of an empty PCL cloud does exactly that.
        sensor_msgs::msg::PointCloud2 empty_cloud;
        pcl::toROSMsg(pcl::PointCloud<pcl::PointXYZ>(), empty_cloud);

        empty_cloud.header.frame_id = frame_id_;
        empty_cloud.header.stamp = now();

        global_map_pub_->publish(empty_cloud);
        processed_pub_->publish(empty_cloud);
        rim_pub_->publish(empty_cloud);
        clusters_pub_->publish(empty_cloud);   // cluster overlay
        last_processed_msg_ = empty_cloud;   // keepalive must not resurrect the old map

        // ---------------------------------------------------------
        // Clear MarkerArray topics
        // ---------------------------------------------------------

        visualization_msgs::msg::MarkerArray clear_markers;

        visualization_msgs::msg::Marker clear;

        clear.action =
            visualization_msgs::msg::Marker::DELETEALL;

        clear_markers.markers.push_back(clear);

        targets_markers_pub_->publish(clear_markers);
        cylinder_marker_pub_->publish(clear_markers);

        // ---------------------------------------------------------
        // Clear mesh marker
        // ---------------------------------------------------------

        visualization_msgs::msg::Marker clear_mesh;

        clear_mesh.header.frame_id = frame_id_;
        clear_mesh.header.stamp = now();
        clear_mesh.action =
            visualization_msgs::msg::Marker::DELETE;

        // Must match loadAndPublishMesh(), or RViz keeps the mesh.
        clear_mesh.ns = "reconstructed_mesh";
        clear_mesh.id = 0;

        mesh_pub_->publish(clear_mesh);

        // ---------------------------------------------------------
        // Clear Path
        // ---------------------------------------------------------

        nav_msgs::msg::Path empty_path;

        empty_path.header.frame_id = frame_id_;
        empty_path.header.stamp = now();

        path_pub_->publish(empty_path); 

        RCLCPP_INFO(
            get_logger(),
            "Previous RViz visualisations cleared.");
    }

    // params
    std::string cloud_topic_, global_map_topic_, processed_topic_, frame_id_, save_path_, mesh_path_, locked_targets_path_, path_topic_, path_yaml_,
                clicked_topic_, rim_topic_, cylinder_marker_topic_, mesh_topic_, targets_topic_, targets_markers_topic_;
    std::string output_dir_, run_stamp_, loaded_mesh_path_;
    std::string body_frame_, lidar_frame_;
    std::filesystem::path run_dir_;

    double voxel_leaf_size_, duplicate_distance_, sor_stddev_mult_;
    bool apply_final_bounds_ = false;
    double final_min_x_, final_max_x_, final_min_y_, final_max_y_, final_min_z_, final_max_z_;
    int sor_mean_k_;
    // Per-scan crop sector in LiDAR-frame spherical coords (constructor).
    double fov_min_range_ = 0.10, fov_max_range_ = 0.60;
    double fov_min_azimuth_deg_ = -20.0, fov_max_azimuth_deg_ = 20.0;
    double fov_min_elevation_deg_ = -20.0, fov_max_elevation_deg_ = 20.0;

    // Derived; updateFovDerived() owns all of these.
    float fov_min_range_sq_ = 0.0f, fov_max_range_sq_ = 0.0f;
    float fov_tan_el_min_ = 0.0f, fov_tan_el_max_ = 0.0f;
    float fov_az_min_f_ = 0.0f, fov_az_span_ = 360.0f;
    bool  fov_az_full_ = false, fov_el_full_ = false;
    int max_ransac_iterations_;
    double min_ransac_radius_, max_ransac_radius_, ransac_probability_;
    double snapshot_period_sec_;
    int snapshot_count_ = 0;
    rclcpp::TimerBase::SharedPtr snapshot_timer_;
    rclcpp::TimerBase::SharedPtr mesh_watch_timer_;


    // state
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    std::mutex map_mutex_;
    bool collecting_ = false;
    bool saved_ = false;
    float last_cylinder_radius_ = 0.0f;
    bool has_cylinder_ = false;
    bool has_locked_target_ = false;
    bool pipeline_done_ = false;
    bool generate_path_ = false;
    int live_mesh_floor_ = 0;   // live_*.stl below this index predate /clear_map


    // ROS: Subscriptions/ publishers
    rclcpp::Time run_start_time_;
    rclcpp::TimerBase::SharedPtr finalize_timer_;                      // replaces collection_timer_

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_, processed_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rim_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cylinder_marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // For cylinder rim and axis visualisation
    Eigen::Vector3f last_cylinder_axis_point_;
    Eigen::Vector3f last_cylinder_axis_dir_;

    // State used by onClickedPoint() and pipeline
    std_msgs::msg::Header last_header_;
    sensor_msgs::msg::PointCloud2 last_processed_msg_;

    // Mesh face geometry for click-to-face-normal lookup and cylinder fit
    pcl::PointCloud<PointT>::Ptr      mesh_centroids_;       // one point per face (centroid)
    pcl::PointCloud<pcl::Normal>::Ptr  mesh_face_normals_;    // one normal per face
    pcl::search::KdTree<PointT>::Ptr  mesh_tree_;            // KD-tree on centroids
    

    // a timer to publish processed result (global map) later (different from collection timer)
    rclcpp::TimerBase::SharedPtr keepalive_timer_;

    // for locked target
    geometry_msgs::msg::PoseStamped locked_target_;

    // ---- TF ----
    bool wait_for_tf_ = true;
    bool have_tf_ = false;
    double tf_lookup_timeout_sec_ = 0.1;
    double tf_buffer_sec_ = 10.0;
    std::size_t scans_dropped_ = 0;

    // ---- Deskew ----
    bool deskew_ = true;
    int  deskew_buckets_ = 10;
    std::size_t deskew_pose_misses_ = 0;

    // ---- Drone trajectory ----
    bool        publish_trajectory_ = true;
    std::string drone_frame_, trajectory_topic_;
    double      trajectory_period_sec_ = 0.05;
    double      trajectory_min_dist_ = 0.005;
    int         trajectory_max_points_ = 20000;

    nav_msgs::msg::Path          trajectory_;            // accumulated, republished whole
    rclcpp::Time                 last_trajectory_stamp_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr trajectory_timer_;

    // drone -> trunk axis midpoint
    bool        publish_offset_ = true;
    std::string offset_topic_;
    double      offset_period_sec_ = 0.05;
    Eigen::Vector3f last_cylinder_mid_ = Eigen::Vector3f::Zero();
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr offset_pub_;
    rclcpp::TimerBase::SharedPtr offset_timer_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr                tf_wait_timer_;

    // for multiple normals
    std::vector<geometry_msgs::msg::PoseStamped> selected_normals_;
    double normal_merge_radius_;          // declare_parameter("normal_merge_radius", 0.03)
    std::string normals_save_path_;       // HOME + "/vision_ws/selected_normals.yaml"
    // Locked normal targets
    std::vector<geometry_msgs::msg::PoseStamped> locked_targets_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr targets_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr targets_markers_pub_;

    // Collection services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_collection_srv_, stop_collection_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_map_srv_;

    // Normal selection service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_selection_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dump_normals_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_normals_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_locked_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_normals_srv_;

    // path services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_path_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_path_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_trajectory_srv_;

    // ---- Cluster removal ----
    double cluster_voxel_        = 0.01;   // leaf of the copy that gets clustered
    double cluster_tolerance_    = 0.05;   // join distance
    int    cluster_min_points_   = 50;     // in voxels, not raw points
    int    cluster_max_points_   = 10000000;
    double cluster_click_radius_ = 0.10;   // a click further away is ignored
    std::string clusters_topic_, cluster_status_topic_;

    bool cluster_armed_   = false;         // clicks pick clusters, not normals
    int  pending_cluster_ = -1;            // -1 = armed but nothing clicked yet

    pcl::PointCloud<PointT>::Ptr    cluster_cloud_;     // the voxelised copy
    pcl::search::KdTree<PointT>::Ptr cluster_tree_;     // KD-tree on that copy
    std::vector<pcl::PointIndices>  cluster_indices_;   // largest cluster first
    std::vector<int>                cluster_label_;     // per copy point, -1 = none

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         cluster_status_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cluster_map_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_cluster_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_cluster_srv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPipeline>()); // executor: single-threaded
    rclcpp::shutdown();
    return 0;
}