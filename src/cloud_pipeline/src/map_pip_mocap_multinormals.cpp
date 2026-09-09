// Map pipeline to work with mocap, for selecting more than 1 normal
//  Using since 18 August 2026
// Author: Thanh Tin Nguyen
// Email: ttn32@cam.ac.uk

// Launch commands:
//chmod +x ~/vision_ws/run_mid360_dlio_mappipmocap.sh
//~/vision_ws/run_mid360_dlio_mappipmocap-poisson.sh

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

#include <nav_msgs/msg/path.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Geometry>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

struct SelectedTarget
{
    pcl::PointXYZ position;
    pcl::Normal normal;
};

class MapPipeline : public rclcpp::Node
{
public:
    MapPipeline() : Node("cloud_accumulator")
    {
        // TOPICS
        //cloud_topic_       = declare_parameter<std::string>("cloud_topic", "/dlio/odom_node/pointcloud/deskewed");
        cloud_topic_       = declare_parameter<std::string>("cloud_topic", "/livox/lidar");
        clicked_topic_     = declare_parameter<std::string>("clicked_topic", "/clicked_point");
        global_map_topic_  = declare_parameter<std::string>("global_map_topic", "/global_map");
        processed_topic_   = declare_parameter<std::string>("processed_topic", "/processed/map");
        normals_topic_     = declare_parameter<std::string>("normals_topic", "/processed/normals");
        clusters_topic_    = declare_parameter<std::string>("clusters_topic", "/processed/clusters");
        //target_topic_      = declare_parameter<std::string>("locked_target", "/processed/locked_target");
        targets_topic_     = declare_parameter<std::string>("targets_topic", "/processed/selected_normals");
        targets_markers_topic_ = declare_parameter<std::string>("targets_markers_topic", "selected_normals_markers");
        path_topic_        = declare_parameter<std::string>("path_topic", "/processed/path");

        rim_topic_         = declare_parameter<std::string>("cylinder_rim", "/processed/cylinder_rim");
        cylinder_marker_topic_= declare_parameter<std::string>("cylinder_axis", "/processed/cylinder_axis");
        mesh_topic_        = declare_parameter<std::string>("mesh_topic", "/processed/mesh");

        // Frames
        frame_id_          = declare_parameter<std::string>("frame_id", "odom");
        world_frame_       = declare_parameter<std::string>("world_frame", "odom");
        lidar_frame_       = declare_parameter<std::string>("lidar_frame", "lidar");
        //deskew_enabled_    = declare_parameter<bool>("deskew_enabled", true);
        tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.1);

        // MEMORIES
        save_path_ = declare_parameter<std::string>("save_path","");
        mesh_path_ = declare_parameter<std::string>("mesh_path","");
        normals_save_path_  = declare_parameter<std::string>("normals_save_path", "");
        locked_targets_path_ = (std::filesystem::path(std::getenv("HOME"))
                                /"vision_ws_outputs/normals/locked_target.yaml").string();

        // ----------------
        // OPERATIONS
        collection_duration_sec_ = declare_parameter<double>("collection_duration_sec", 15.0);
        use_previous_normal_     = declare_parameter<bool>("use_previous_normal", false);
        generate_path_           = declare_parameter<bool>("generate_path", false);
        path_yaml_               =declare_parameter<std::string>("path_yaml", "");
        normal_merge_radius_     = declare_parameter<double>("normal_merge_radius", 0.01);


        // downsampling parameters
        min_x_ = declare_parameter<double>("min_x", 0);
        max_x_ = declare_parameter<double>("max_x",  0.5);

        min_y_ = declare_parameter<double>("min_y", -0.2);
        max_y_ = declare_parameter<double>("max_y",  0.2);

        min_z_ = declare_parameter<double>("min_z", 0);
        max_z_ = declare_parameter<double>("max_z",  0.2);

        voxel_leaf_size_        = declare_parameter<double>("voxel_leaf_size", 0.003);
        duplicate_distance_     = declare_parameter<double>("duplicate_distance", 0.001);

        // Normal estimation parameters
        sor_mean_k_             = declare_parameter<int>("sor_mean_k", 1000);
        sor_stddev_mult_        = declare_parameter<double>("sor_stddev_mult", 10.0);
        normal_k_search_        = declare_parameter<int>("normal_k_search", 30);
        mls_poly_               = declare_parameter<int>("mls_poly", 2);
        mls_search_radius_      = declare_parameter<double>("mls_search_radius", 0.03);

        marker_scale_           = declare_parameter<double>("marker_scale", 0.05);
        marker_stride_          = declare_parameter<int>("marker_stride", 5);

        // Clustering parameters
        region_growing_neighbors_ = declare_parameter<int>("region_growing_neighbors", 30);
        smoothness_threshold_     = declare_parameter<double>("smoothness_threshold", 7.0 / 180.0 * M_PI);
        curvature_threshold_      = declare_parameter<double>("curvature_threshold", 1.0);

        // Cylinder RANSAC parameters
        max_ransac_iterations_        = declare_parameter<int>("max_ransac_iterations", 1000);
        min_ransac_radius_            = declare_parameter<double>("min_radius", 0.005);
        max_ransac_radius_            = declare_parameter<double>("max_radius", 0.15);
        ransac_probability_           = declare_parameter<double>("ransac_probability", 0.95);

        // Verify collection_duration
        if (collection_duration_sec_ <= 0.0)
        {
            RCLCPP_WARN(get_logger(),
                "collection_duration_sec must be > 0, got %.2f. Defaulting to 10.0s.",
                collection_duration_sec_);
            collection_duration_sec_ = 10.0;
        }

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

        normals_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            normals_topic_, rclcpp::QoS(1).transient_local());

        cluster_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            clusters_topic_, rclcpp::QoS(1).transient_local());

        //target_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(    target_topic_ , rclcpp::QoS(1).transient_local());

        targets_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            targets_topic_ , rclcpp::QoS(1).transient_local());

        targets_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        targets_markers_topic_, rclcpp::QoS(1).transient_local());

        rim_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            rim_topic_, rclcpp::QoS(1).transient_local());

        cylinder_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            cylinder_marker_topic_, rclcpp::QoS(1).transient_local());

        mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            mesh_topic_, rclcpp::QoS(1).transient_local());

        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            path_topic_, rclcpp::QoS(1).transient_local());

        // initialise buffer/ listener

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        auto period =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(collection_duration_sec_));

        // create collection timer: waite for period_, then calll onCollectionComplete when done
        collection_timer_ = create_wall_timer(
            period,
            std::bind(&MapPipeline::onCollectionComplete, this));


        // Services for multiple normals - These services allow user to manage selected normal from ROS2 command.


        finish_selection_srv_ =
            this->create_service<std_srvs::srv::Trigger>(
                "finish_normal_selection",
                std::bind(
                    &MapPipeline::finishNormalSelection,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        load_path_srv_ =
            this->create_service<std_srvs::srv::Trigger>(
                "load_and_publish_path",
                std::bind(
                    &MapPipeline::loadAndPublishPathService,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));


        RCLCPP_INFO(get_logger(),
            "cloud_accumulator: collecting from '%s' for %.1f seconds...",
            cloud_topic_.c_str(), collection_duration_sec_);

    }

    // destructor + early termination: two ways to save: normal completion / early shutdown
    // destructor: before MapPipeLIne object disappears, do a final cleanup/ finalisation work.
    ~MapPipeline()
    {
        // Fallback path: only relevant if the node is killed early (Ctrl+C)
        // before the collection window elapses. If onCollectionComplete()
        // already ran, saveMap() below is a no-op thanks to saved_.
        saveMap();
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!collecting_) return;

        // receiving point cloud to be Ptr scan.
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *scan);
        if (scan->empty()) return;

        // BOUND: range filter into a sane working box
        pcl::PointCloud<pcl::PointXYZ>::Ptr bounded_scan(new pcl::PointCloud<pcl::PointXYZ>);

        bounded_scan->reserve(scan->size());

        for (const auto &p : scan->points)
        {
            if (p.x >= min_x_ && p.x <= max_x_ &&
                p.y >= min_y_ && p.y <= max_y_ &&
                p.z >= min_z_ && p.z <= max_z_)
            {
                bounded_scan->push_back(p); // add point into scan of reserved size.
            }
        }

        // TRANSFORM: lidar -> world:
        // at the exact time this LiDAR scan was taken, ask TF2 where
        // the LiDAR was relative to my world frame.

        geometry_msgs::msg::TransformStamped transform; // variable to hold the transform

        try // possible errors:world doesn't exist,
            // lidar doesn't exist,
            // no pose from MOCAP,
            // timestamp doesn't line up,
            // TF data unavailable
        {
            transform = tf_buffer_->lookupTransform(
                frame_id_,                 // target = world/ odom
                msg->header.frame_id,      // source = lidar
                msg->header.stamp,
                tf2::durationFromSec(tf_lookup_timeout_sec_)); // ask TF2 for the transform. wait up to 100ms for TF2 to reply
        }
        catch (const tf2::TransformException &ex) // if TF2 data unavailable
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Could not transform %s -> %s: %s",
                msg->header.frame_id.c_str(),
                frame_id_.c_str(),
                ex.what());

            return; // discard this scan instead of wrongly transform it.
        }


        // Convert TF -> Eigen
        Eigen::Isometry3d T_world_lidar =
            tf2::transformToEigen(transform); // convert the trasnform into eigen (4x4 homo transform matrix)


        // create a transformed cloud in world frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr world_scan(
            new pcl::PointCloud<pcl::PointXYZ>);

        pcl::transformPointCloud(
            *bounded_scan,
            *world_scan,
            T_world_lidar.cast<float>().matrix());

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
            "Scan: %zu points | New: %zu | Map: %zu",
            world_scan->size(),
            new_points->size(),
            global_map_->size());

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

    // ---- fires exactly once, when the collection window elapses ----
    void onCollectionComplete()
    {
        collection_timer_->cancel();   // stop this timer from firing again
        collecting_ = false; // stop collecting
        cloud_sub_.reset();            // fully unsubscribe, stop consuming new scans

        // BOUND: final range filter to filter out positive values only.
        pcl::PointCloud<pcl::PointXYZ>::Ptr bounded_global(new pcl::PointCloud<pcl::PointXYZ>);

        bounded_global->reserve(global_map_->size());

        // took a bit of time to fix this. essentially because of drift, if box is too small,
        // it can remove the entire global map.
        for (const auto &p : global_map_->points)
        {
            if (p.x >= 0.1 && p.x <= 0.5 && // limit x to front
                p.y >= -0.5 && p.y <= 0.5 &&
                p.z >= -1.0 && p.z <= 1.0) // limit
            {
                bounded_global->push_back(p); // add point into scan of reserved size.
            }
        }

        global_map_ = bounded_global; // reassign global_map_ to the final bounded form

        // DENOISE: denoise the entire cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr denoised (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(global_map_);
        sor.setMeanK(sor_mean_k_);
        sor.setStddevMulThresh(sor_stddev_mult_);
        sor.filter(*denoised);
        if (global_map_->empty())
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
            "Collection window complete (%.1fs). Final map: %zu points. Running pipeline...",
            collection_duration_sec_, final_count);

        saveMap();

        if (use_previous_normal_)
        {
            RCLCPP_INFO(
                get_logger(),
                "User selected previous normals.");

            if (!loadLockedTargets())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to load previous locked normals.");
            }
            else
            {
                publishLockedTargets();
            }

            // Important:
            // Do NOT run new normal estimation in this mode.
        }
        else
        {
            RCLCPP_INFO(
                get_logger(),
                "User selected new normal estimation.");

            runNormalEstimationMLS();
        }

        publishMesh(mesh_path_);

        // keep republishing processed results 1Hz after pipeline finished.
        // [this]() defining a small lambda function. if pipeline is not finished, do nothing
        // if finished, publish results.

        keepalive_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            if (!pipeline_done_) return;
            processed_pub_->publish(last_processed_msg_);
            normals_pub_->publish(last_normals_msg_);

            // CAUTION: will need to keep publishing clusters,
            // targets, cylinder fit as well
        });

        // Generate path is done by service call, not automatically after normal estimation.

        RCLCPP_INFO(get_logger(),
            "Pipeline complete. '%s' and '%s' are latched — inspect in RViz2, or Ctrl+C to exit.",
            processed_topic_.c_str(), normals_topic_.c_str());
    }

    // ---- MAIN PROCESSING PIPELINE expensive stages, run once on the final collected map ----
    void runNormalEstimation()
    {

        // NORMAL ESTIMATION BLOCK:
        // using planeSVD kNN search tree (could consider switching to MLS)

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        //ne.setInputCloud(global_map);
        ne.setInputCloud(global_map_);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setNumberOfThreads(4);
        ne.setSearchMethod(tree);
        ne.setKSearch(normal_k_search_);
        ne.compute(*normals);


        RCLCPP_INFO(get_logger(), "Normals computed: %zu points -> %zu normals",
                    global_map_->size(), normals->size());

        auto stamp = get_clock()->now();

        // NormalEstimation is one-to-one with the input cloud by index, but points
        // with too few valid neighbors come back with a NaN normal — strip those
        // out, keeping position and normal in lockstep.
        pcl::PointCloud<PointT>::Ptr seg_input(new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr seg_normals(new pcl::PointCloud<pcl::Normal>);
        seg_input->points.reserve(global_map_->size());
        seg_normals->points.reserve(global_map_->size());
        for (size_t i = 0; i < normals->points.size(); ++i) {
        const auto& n = normals->points[i];
        if (std::isfinite(n.normal_x) && std::isfinite(n.normal_y) && std::isfinite(n.normal_z)) {
            seg_input->points.push_back(global_map_->points[i]);
            seg_normals->points.push_back(n);
        }
        }
        seg_input->width = seg_input->points.size();
        seg_input->height = 1;
        seg_normals->width = seg_normals->points.size();
        seg_normals->height = 1;

        if (seg_input->empty()) return;

        RCLCPP_INFO(get_logger(), "Normal estimation: %zu valid points (from %zu input points)",
                seg_input->size(), global_map_->size());

        // REGION GROWING SEGMENTATION
        pcl::search::KdTree<PointT>::Ptr seg_tree(new pcl::search::KdTree<PointT>());

        pcl::RegionGrowing<PointT, pcl::Normal> reg;
        reg.setMinClusterSize(20);
        reg.setMaxClusterSize(100000);
        reg.setSearchMethod(seg_tree);
        reg.setNumberOfNeighbours(region_growing_neighbors_); // TUNE: number of neighbors to analyze for each point
        reg.setInputCloud(seg_input);
        reg.setInputNormals(seg_normals);
        reg.setSmoothnessThreshold(smoothness_threshold_);  // TUNE ~3 degree — how "equal" normals must be
        reg.setCurvatureThreshold(curvature_threshold_);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        RCLCPP_INFO(get_logger(), "Found %zu normal-consistent clusters", clusters.size());

        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(*global_map_, out);
        out.header.frame_id = frame_id_;
        out.header.stamp = stamp;
        last_processed_msg_ = out;      // cache for keepalive republish
        processed_pub_->publish(out);

        std_msgs::msg::Header hdr;
        hdr.frame_id = frame_id_;
        hdr.stamp = stamp;

        // --- Publish normals as MarkerArray ---
        //publishNormalMarkers(mls_points, msg->header);
        publishNormalMarkers(seg_input, seg_normals, hdr.stamp);

        // --- Publish clusters as colored point cloud ---
        publishClusters(seg_input, clusters, hdr);

        // --- Publish cylinder fit ---
        t.start();
        fitAndPublishCylinder(seg_input, seg_normals, hdr);
        RCLCPP_INFO(get_logger(), "[TIMER] Cylinder Fitting: %.2f ms", t.stop_ms());

        // --- Store latest frame's data for the click handler ---
        last_cloud_ = seg_input;
        last_normals_ = seg_normals;
        //last_normals_ = last_normals_;  // Use the normals from MLS, not the Region Growing normals
        last_clusters_ = clusters;
        last_header_.stamp = stamp;
        last_header_.frame_id = frame_id_;

        RCLCPP_INFO(get_logger(), "------------------------");
        // TODO: need
        pipeline_done_ = true;
    }

    void runNormalEstimationMLS()
    {
        // ============================================================
        // MLS: Moving Least Squares smoothing + normal estimation
        // ============================================================

        pcl::search::KdTree<PointT>::Ptr tree(
            new pcl::search::KdTree<PointT>());

        pcl::PointCloud<pcl::PointNormal>::Ptr mls_output(
            new pcl::PointCloud<pcl::PointNormal>);

        pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;

        mls.setInputCloud(global_map_);
        mls.setPolynomialOrder(mls_poly_);
        mls.setSearchMethod(tree);

        // Compute normals
        mls.setComputeNormals(true);

        // Radius controls how much neighbouring data is used
        // for the local surface fit.
        mls.setSearchRadius(mls_search_radius_);

        mls.process(*mls_output);

        // Flip all normals
        for (auto& n : mls_output->points) {
            n.normal_x *= -1.0f;
            n.normal_y *= -1.0f;
            n.normal_z *= -1.0f;
        }

        if (mls_output->empty()) {
            RCLCPP_ERROR(
                get_logger(),
                "MLS produced no output points.");
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "MLS computed: %zu input points -> %zu output points",
            global_map_->size(),
            mls_output->size());


        // ============================================================
        // Remove points with invalid normals
        // ============================================================

        pcl::PointCloud<PointT>::Ptr seg_input(
            new pcl::PointCloud<PointT>);

        pcl::PointCloud<pcl::Normal>::Ptr seg_normals(
            new pcl::PointCloud<pcl::Normal>);

        seg_input->points.reserve(mls_output->size());
        seg_normals->points.reserve(mls_output->size());

        for (const auto& p : mls_output->points)
        {
            if (std::isfinite(p.x) &&
                std::isfinite(p.y) &&
                std::isfinite(p.z) &&
                std::isfinite(p.normal_x) &&
                std::isfinite(p.normal_y) &&
                std::isfinite(p.normal_z))
            {
                PointT point;

                point.x = p.x;
                point.y = p.y;
                point.z = p.z;

                seg_input->points.push_back(point);

                pcl::Normal normal;

                normal.normal_x = p.normal_x;
                normal.normal_y = p.normal_y;
                normal.normal_z = p.normal_z;
                normal.curvature = p.curvature;

                seg_normals->points.push_back(normal);
            }
        }

        seg_input->width = seg_input->points.size();
        seg_input->height = 1;
        seg_input->is_dense = true;

        seg_normals->width = seg_normals->points.size();
        seg_normals->height = 1;
        seg_normals->is_dense = true;

        if (seg_input->empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "No valid MLS points/normals remained.");
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "MLS: %zu valid points from %zu output points",
            seg_input->size(),
            mls_output->size());


        // ============================================================
        // Region Growing Segmentation
        // ============================================================

        pcl::search::KdTree<PointT>::Ptr seg_tree(
            new pcl::search::KdTree<PointT>());

        pcl::RegionGrowing<PointT, pcl::Normal> reg;

        reg.setMinClusterSize(20);
        reg.setMaxClusterSize(100000);

        reg.setSearchMethod(seg_tree);
        reg.setNumberOfNeighbours(region_growing_neighbors_);

        reg.setInputCloud(seg_input);
        reg.setInputNormals(seg_normals);

        reg.setSmoothnessThreshold(
            smoothness_threshold_);

        reg.setCurvatureThreshold(
            curvature_threshold_);

        std::vector<pcl::PointIndices> clusters;

        reg.extract(clusters);

        RCLCPP_INFO(
            get_logger(),
            "Found %zu normal-consistent clusters",
            clusters.size());


        // ============================================================
        // Publish processed cloud
        // ============================================================

        auto stamp = get_clock()->now();

        sensor_msgs::msg::PointCloud2 out;

        pcl::toROSMsg(*seg_input, out);

        out.header.frame_id = frame_id_;
        out.header.stamp = stamp;

        last_processed_msg_ = out;

        processed_pub_->publish(out);


        // ============================================================
        // Header
        // ============================================================

        std_msgs::msg::Header hdr;

        hdr.frame_id = frame_id_;
        hdr.stamp = stamp;


        // ============================================================
        // Publish normals
        // ============================================================

        publishNormalMarkers(
            seg_input,
            seg_normals,
            hdr.stamp);


        // ============================================================
        // Publish clusters
        // ============================================================

        publishClusters(
            seg_input,
            clusters,
            hdr);


        // ============================================================
        // Cylinder fitting
        // ============================================================

        t.start();

        fitAndPublishCylinder(
            seg_input,
            seg_normals,
            hdr);

        RCLCPP_INFO(
            get_logger(),
            "[TIMER] Cylinder Fitting: %.2f ms",
            t.stop_ms());


        // ============================================================
        // Store data for click handler
        // ============================================================

        last_cloud_ = seg_input;
        last_normals_ = seg_normals;
        last_clusters_ = clusters;

        last_header_.stamp = stamp;
        last_header_.frame_id = frame_id_;


        RCLCPP_INFO(
            get_logger(),
            "------------------------");

        pipeline_done_ = true;
    }

    bool loadLockedTargets()
    {
        locked_targets_.clear();

        if (locked_targets_path_.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "locked_targets_path is empty.");
            return false;
        }

        if (!std::filesystem::exists(locked_targets_path_))
        {
            RCLCPP_WARN(
                get_logger(),
                "Locked target file does not exist: %s",
                locked_targets_path_.c_str());
            return false;
        }

        try
        {
            YAML::Node root =
                YAML::LoadFile(locked_targets_path_);

            if (!root["targets"] ||
                !root["targets"].IsSequence())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Invalid locked target YAML: "
                    "'targets' is missing or not a sequence.");
                return false;
            }

            std::string frame =
                root["frame_id"]
                    ? root["frame_id"].as<std::string>()
                    : frame_id_;

            for (const auto &node : root["targets"])
            {
                geometry_msgs::msg::PoseStamped target;

                target.header.frame_id = frame;
                target.header.stamp = get_clock()->now();

                target.pose.position.x =
                    node["position"]["x"].as<double>();

                target.pose.position.y =
                    node["position"]["y"].as<double>();

                target.pose.position.z =
                    node["position"]["z"].as<double>();

                target.pose.orientation.x =
                    node["orientation"]["x"].as<double>();

                target.pose.orientation.y =
                    node["orientation"]["y"].as<double>();

                target.pose.orientation.z =
                    node["orientation"]["z"].as<double>();

                target.pose.orientation.w =
                    node["orientation"]["w"].as<double>();

                locked_targets_.push_back(target);
            }

            RCLCPP_INFO(
                get_logger(),
                "Loaded %zu locked normals from %s",
                locked_targets_.size(),
                locked_targets_path_.c_str());

            return !locked_targets_.empty();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to load locked normals: %s",
                e.what());

            return false;
        }
    }

    void publishLockedTargets()
    {
        geometry_msgs::msg::PoseArray pa;

        pa.header.frame_id = frame_id_;
        pa.header.stamp = get_clock()->now();

        for (const auto& target : locked_targets_)
        {
            pa.poses.push_back(target.pose);
        }

        targets_pub_->publish(pa);

        visualization_msgs::msg::MarkerArray ma;

        // Clear previous selected/locked normal markers
        visualization_msgs::msg::Marker wipe;
        wipe.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(wipe);

        int id = 0;

        for (size_t i = 0;
            i < locked_targets_.size();
            ++i)
        {
            const auto& pose =
                locked_targets_[i].pose;

            Eigen::Quaternionf q(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z);

            Eigen::Vector3f n =
                q * Eigen::Vector3f(1, 0, 0);

            visualization_msgs::msg::Marker arrow;

            arrow.header.frame_id = frame_id_;
            arrow.header.stamp = get_clock()->now();

            arrow.ns = "locked_normals";
            arrow.id = id++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point p0, p1;

            p0.x = pose.position.x;
            p0.y = pose.position.y;
            p0.z = pose.position.z;

            p1.x = p0.x + n.x() * 0.08;
            p1.y = p0.y + n.y() * 0.08;
            p1.z = p0.z + n.z() * 0.08;

            arrow.points = {p0, p1};

            arrow.scale.x = 0.004;
            arrow.scale.y = 0.008;
            arrow.scale.z = 0.012;

            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.color.a = 1.0f;

            arrow.pose.orientation.w = 1.0;

            ma.markers.push_back(arrow);

            visualization_msgs::msg::Marker txt;

            txt.header = arrow.header;
            txt.ns = "locked_normals_label";
            txt.id = id++;
            txt.type =
                visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::msg::Marker::ADD;

            txt.pose.position.x = pose.position.x;
            txt.pose.position.y = pose.position.y;
            txt.pose.position.z = pose.position.z + 0.03;

            txt.pose.orientation.w = 1.0;

            txt.scale.z = 0.02;

            txt.color.r = 1.0f;
            txt.color.g = 1.0f;
            txt.color.b = 1.0f;
            txt.color.a = 1.0f;

            txt.text = std::to_string(i);

            ma.markers.push_back(txt);
        }

        targets_markers_pub_->publish(ma);
    }

    // publish clusters of similar normals as colored point clouds for RViz2
    void publishClusters(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        const std::vector<pcl::PointIndices>& clusters,
        const std_msgs::msg::Header& header)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (const auto& cluster : clusters) {
            // Compute this cluster's centroid
            float cx = 0, cy = 0, cz = 0;
            for (int idx : cluster.indices) {
            cx += cloud->points[idx].x;
            cy += cloud->points[idx].y;
            cz += cloud->points[idx].z;
            }
            size_t n = cluster.indices.size();
            cx /= n; cy /= n; cz /= n;

            uint8_t r, g, b;
            colorFromPosition(cx, cy, cz, r, g, b);

            for (int idx : cluster.indices) {
            pcl::PointXYZRGB pt;
            pt.x = cloud->points[idx].x;
            pt.y = cloud->points[idx].y;
            pt.z = cloud->points[idx].z;
            pt.r = r; pt.g = g; pt.b = b;
            colored->points.push_back(pt);
            }
        }
        colored->width = colored->points.size();
        colored->height = 1;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*colored, msg);
        msg.header = header;
        cluster_pub_->publish(msg);
    }


    void publishNormalMarkers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        const rclcpp::Time &stamp)
    {
        visualization_msgs::msg::MarkerArray array;

        // Clear all previously displayed normal markers
        visualization_msgs::msg::Marker wipe;
        wipe.action = visualization_msgs::msg::Marker::DELETEALL;
        array.markers.push_back(wipe);

        int stride = std::max(1, marker_stride_);
        int id = 0;

        for (size_t i = 0;
            i < cloud->size();
            i += static_cast<size_t>(stride))
        {
            const auto &p = cloud->points[i];
            const auto &n = normals->points[i];

            if (!std::isfinite(n.normal_x) ||
                !std::isfinite(n.normal_y) ||
                !std::isfinite(n.normal_z))
            {
                continue;
            }

            visualization_msgs::msg::Marker arrow;

            arrow.header.frame_id = frame_id_;
            arrow.header.stamp = stamp;

            arrow.ns = "map_normals";
            arrow.id = id++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point p0, p1;

            p0.x = p.x;
            p0.y = p.y;
            p0.z = p.z;

            p1.x = p.x + n.normal_x * marker_scale_;
            p1.y = p.y + n.normal_y * marker_scale_;
            p1.z = p.z + n.normal_z * marker_scale_;

            arrow.points.push_back(p0);
            arrow.points.push_back(p1);

            arrow.scale.x = 0.002;
            arrow.scale.y = 0.004;
            arrow.scale.z = 0.006;

            arrow.color.r = 1.0f;
            arrow.color.g = 0.6f;
            arrow.color.b = 0.0f;
            arrow.color.a = 1.0f;

            arrow.pose.orientation.w = 1.0;

            array.markers.push_back(arrow);
        }

        last_normals_msg_ = array;
        normals_pub_->publish(array);
    }

    void publishMesh(const std::string& stl_path)
    {

        const int max_wait_seconds = 30;

        for (int i = 0; i < max_wait_seconds; ++i)
        {
            if (std::filesystem::exists(stl_path))
            {
                break;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        if (!std::filesystem::exists(stl_path))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Mesh file was not created within %d seconds: %s",
                max_wait_seconds,
                stl_path.c_str());
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Mesh found! Publishing: %s",
            stl_path.c_str());


        visualization_msgs::msg::Marker mesh;

        mesh.header.frame_id = frame_id_;
        mesh.header.stamp = get_clock()->now();

        mesh.ns = "reconstructed_mesh";
        mesh.id = 0;

        mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        mesh.action = visualization_msgs::msg::Marker::ADD;

        // STL file URI
        mesh.mesh_resource = "file://" + stl_path;

        // Position
        mesh.pose.position.x = 0.0;
        mesh.pose.position.y = 0.0;
        mesh.pose.position.z = 0.0;

        // No rotation
        mesh.pose.orientation.x = 0.0;
        mesh.pose.orientation.y = 0.0;
        mesh.pose.orientation.z = 0.0;
        mesh.pose.orientation.w = 1.0;

        // Scale
        mesh.scale.x = 1.0;
        mesh.scale.y = 1.0;
        mesh.scale.z = 1.0;

        // Make mesh visible
        mesh.color.r = 0.5f;
        mesh.color.g = 0.5f;
        mesh.color.b = 0.5f;
        mesh.color.a = 0.7;

        mesh_pub_->publish(mesh);

        RCLCPP_INFO(
            get_logger(),
            "Published reconstructed mesh: %s",
            stl_path.c_str());
    }

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

        if (path_yaml_.empty())
        {
            response->success = false;
            response->message = "path_yaml parameter is empty.";
            return;
        }

        if (!std::filesystem::exists(path_yaml_))
        {
            response->success = false;
            response->message =
                "Path YAML does not exist: " + path_yaml_;
            return;
        }

        bool success = loadAndPublishPath(path_yaml_);

        response->success = success;

        if (success)
        {
            response->message =
                "Path loaded and published: " + path_yaml_;
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

            /* this block creates marker path instead of path path

            visualization_msgs::msg::MarkerArray markers;

            // ---------------------------------------------------------
            // Clear any previous path
            // ---------------------------------------------------------

            visualization_msgs::msg::Marker wipe;

            wipe.header.frame_id = frame;
            wipe.header.stamp = get_clock()->now();

            wipe.ns = "generated_path";
            wipe.id = 0;
            wipe.action =
                visualization_msgs::msg::Marker::DELETEALL;

            markers.markers.push_back(wipe);

            // ---------------------------------------------------------
            // Path line
            // ---------------------------------------------------------

            visualization_msgs::msg::Marker line;

            line.header.frame_id = frame;
            line.header.stamp = get_clock()->now();

            line.ns = "generated_path";
            line.id = 1;

            line.type =
                visualization_msgs::msg::Marker::LINE_STRIP;

            line.action =
                visualization_msgs::msg::Marker::ADD;

            line.pose.orientation.w = 1.0;

            // Thickness of path
            line.scale.x = 0.008;

            // Path colour
            line.color.r = 0.0f;
            line.color.g = 0.0f;
            line.color.b = 1.0f;
            line.color.a = 1.0f;

            // ---------------------------------------------------------
            // Waypoints
            // ---------------------------------------------------------

            for (size_t i = 0; i < waypoints.size(); ++i)
            {
                const auto& waypoint = waypoints[i];

                geometry_msgs::msg::Point p;

                p.x = waypoint["position"]["x"].as<double>();
                p.y = waypoint["position"]["y"].as<double>();
                p.z = waypoint["position"]["z"].as<double>();

                line.points.push_back(p);

                // -----------------------------------------------------
                // Waypoint sphere
                // -----------------------------------------------------

                visualization_msgs::msg::Marker sphere;

                sphere.header = line.header;

                sphere.ns = "generated_path_waypoints";

                sphere.id =
                    static_cast<int>(i);

                sphere.type =
                    visualization_msgs::msg::Marker::SPHERE;

                sphere.action =
                    visualization_msgs::msg::Marker::ADD;

                sphere.pose.position = p;
                sphere.pose.orientation.w = 1.0;

                sphere.scale.x = 0.015;
                sphere.scale.y = 0.015;
                sphere.scale.z = 0.015;

                sphere.color.r = 1.0f;
                sphere.color.g = 0.0f;
                sphere.color.b = 0.0f;
                sphere.color.a = 1.0f;

                markers.markers.push_back(sphere);


                // -----------------------------------------------------
                // Waypoint orientation arrow
                // -----------------------------------------------------

                visualization_msgs::msg::Marker arrow;

                arrow.header = line.header;

                arrow.ns = "generated_path_orientation";

                arrow.id =
                    static_cast<int>(i);

                arrow.type =
                    visualization_msgs::msg::Marker::ARROW;

                arrow.action =
                    visualization_msgs::msg::Marker::ADD;

                arrow.pose.position = p;

                arrow.pose.orientation.x =
                    waypoint["orientation"]["x"].as<double>();

                arrow.pose.orientation.y =
                    waypoint["orientation"]["y"].as<double>();

                arrow.pose.orientation.z =
                    waypoint["orientation"]["z"].as<double>();

                arrow.pose.orientation.w =
                    waypoint["orientation"]["w"].as<double>();

                // Arrow points along local +X
                arrow.scale.x = 0.04;  // length
                arrow.scale.y = 0.006; // shaft diameter
                arrow.scale.z = 0.006; // head diameter

                arrow.color.r = 0.0f;
                arrow.color.g = 1.0f;
                arrow.color.b = 0.0f;
                arrow.color.a = 0.8f;

                markers.markers.push_back(arrow);
            }

            markers.markers.push_back(line);
            */

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

    void onClickedPoint(
        const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // ---------------------------------------------------------
        // 0. Only allow clicks after the pipeline has finished
        // ---------------------------------------------------------
        if (!pipeline_done_ || collecting_)
        {
            RCLCPP_WARN(
                get_logger(),
                "Pipeline not ready for clicks.");
            return;
        }

        // ---------------------------------------------------------
        // 1. Make sure we have cloud + normals + clusters
        // ---------------------------------------------------------
        if (last_clusters_.empty() ||
            !last_cloud_ ||
            !last_normals_)
        {
            RCLCPP_WARN(
                get_logger(),
                "No cluster data yet, ignoring click");
            return;
        }

        // ---------------------------------------------------------
        // 2. Find nearest point to RViz click
        // ---------------------------------------------------------
        double best_dist2 =
            std::numeric_limits<double>::max();

        int best_idx = -1;

        for (size_t i = 0; i < last_cloud_->size(); ++i)
        {
            const auto& p = last_cloud_->points[i];

            double dx = p.x - msg->point.x;
            double dy = p.y - msg->point.y;
            double dz = p.z - msg->point.z;

            double d2 =
                dx * dx +
                dy * dy +
                dz * dz;

            if (d2 < best_dist2)
            {
                best_dist2 = d2;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx < 0)
            return;

        // ---------------------------------------------------------
        // 3. Get clicked point + corresponding normal
        // ---------------------------------------------------------
        const auto& clicked_point =
            last_cloud_->points[best_idx];

        const auto& normal =
            last_normals_->points[best_idx];

        // ---------------------------------------------------------
        // 4. Validate normal
        // ---------------------------------------------------------
        if (!std::isfinite(normal.normal_x) ||
            !std::isfinite(normal.normal_y) ||
            !std::isfinite(normal.normal_z))
        {
            RCLCPP_WARN(
                get_logger(),
                "Clicked point has invalid normal, ignoring click");
            return;
        }

        Eigen::Vector3f normal_vec =
            normal.getNormalVector3fMap();

        if (normal_vec.norm() < 1e-6f)
        {
            RCLCPP_WARN(
                get_logger(),
                "Clicked point has zero-length normal, ignoring click");
            return;
        }

        normal_vec.normalize();

        // ---------------------------------------------------------
        // 5. Print selection information
        // ---------------------------------------------------------
        RCLCPP_INFO(
            get_logger(),
            "Clicked point: (%.3f, %.3f, %.3f), "
            "normal: (%.3f, %.3f, %.3f), distance: %.4f m",
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

            target.header = last_header_;

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

    void finishNormalSelection(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!pipeline_done_) {
        response->success = false;
        response->message =
            "Normal estimation is not finished yet.";
        return;
    }

    if (collecting_) {
        response->success = false;
        response->message =
            "Point-cloud collection is still running.";
        return;
    }

    if (selected_normals_.empty()) {
        response->success = false;
        response->message =
            "No normals have been selected.";
        return;
    }

    if (!saveSelectedNormals()) {
        response->success = false;
        response->message =
            "Failed to save selected normals.";
        return;
    }

    response->success = true;
    response->message =
        "Selected normals saved successfully.";
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

    bool saveSelectedNormals()
    {
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

            std::ofstream fout(normals_save_path_);

            if (!fout.is_open()) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Could not open normals file: %s",
                    normals_save_path_.c_str());

                return false;
            }

            fout << root;
            fout.close();

            RCLCPP_INFO(
                this->get_logger(),
                "Saved %zu selected normals to %s",
                selected_normals_.size(),
                normals_save_path_.c_str());

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

    // Deterministic color from a 3D position — same position always gives same color
    void colorFromPosition(float x, float y, float z, uint8_t& r, uint8_t& g, uint8_t& b)
    {
        // Quantize position to reduce sensitivity to tiny frame-to-frame jitter
        auto quantize = [](float v, float step) {
            return static_cast<int64_t>(std::round(v / step));
        };
        int64_t qx = quantize(x, 0.04f);  // 4cm buckets — tune to your noise level
        int64_t qy = quantize(y, 0.04f);
        int64_t qz = quantize(z, 0.04f);

        // Simple integer hash combining the three quantized coords
        uint64_t h = static_cast<uint64_t>(qx) * 73856093u
                    ^ static_cast<uint64_t>(qy) * 19349663u
                    ^ static_cast<uint64_t>(qz) * 83492791u;

        // Spread hash bits into RGB, keep values in a visible mid-high range
        r = static_cast<uint8_t>(50 + (h % 206));
        g = static_cast<uint8_t>(50 + ((h >> 8) % 206));
        b = static_cast<uint8_t>(50 + ((h >> 16) % 206));
    }

    // RANSAC + Cylinder fitting
    void fitAndPublishCylinder(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr& normals,
        const std_msgs::msg::Header& header)
    {
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
        last_cylinder_axis_point_ = axis_point;
        last_cylinder_axis_dir_   = axis_dir;
        last_cylinder_radius_     = radius;
        has_cylinder_             = true;
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

    // Clear all RViz visualizations (point clouds, markers, mesh) before starting a new collection or after saving the map
     void clearRVizVisualizations()
    {
        RCLCPP_INFO(
            get_logger(),
            "Clearing previous RViz visualisations...");

        // ---------------------------------------------------------
        // Clear global map
        // ---------------------------------------------------------

        sensor_msgs::msg::PointCloud2 empty_cloud;

        empty_cloud.header.frame_id = frame_id_;
        empty_cloud.header.stamp = now();

        empty_cloud.height = 1;
        empty_cloud.width = 0;
        empty_cloud.is_dense = true;

        global_map_pub_->publish(empty_cloud);
        processed_pub_->publish(empty_cloud);
        cluster_pub_->publish(empty_cloud);
        rim_pub_->publish(empty_cloud);

        // ---------------------------------------------------------
        // Clear MarkerArray topics
        // ---------------------------------------------------------

        visualization_msgs::msg::MarkerArray clear_markers;

        visualization_msgs::msg::Marker clear;

        clear.action =
            visualization_msgs::msg::Marker::DELETEALL;

        clear_markers.markers.push_back(clear);

        normals_pub_->publish(clear_markers);
        targets_markers_pub_->publish(clear_markers);
        cylinder_marker_pub_->publish(clear_markers);

        // ---------------------------------------------------------
        // Clear mesh marker
        // ---------------------------------------------------------

        visualization_msgs::msg::Marker clear_mesh;

        clear_mesh.action =
            visualization_msgs::msg::Marker::DELETE;

        clear_mesh.ns = "mesh";
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
    std::string cloud_topic_, global_map_topic_, processed_topic_, normals_topic_, frame_id_, save_path_, mesh_path_, locked_targets_path_, path_topic_, path_yaml_,
                clicked_topic_, clusters_topic_, target_topic_, rim_topic_, cylinder_marker_topic_, mesh_topic_, targets_topic_, targets_markers_topic_;
    std::string world_frame_, lidar_frame_;

    double voxel_leaf_size_, duplicate_distance_, sor_stddev_mult_, marker_scale_, collection_duration_sec_;
    double tf_lookup_timeout_sec_;
    int sor_mean_k_, normal_k_search_, marker_stride_, mls_poly_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
    int region_growing_neighbors_;
    double smoothness_threshold_, curvature_threshold_, mls_search_radius_;
    int max_ransac_iterations_;
    double min_ransac_radius_, max_ransac_radius_, ransac_probability_;


    // state
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    std::mutex map_mutex_;
    bool collecting_ = true;
    bool saved_ = false;
    float last_cylinder_radius_ = 0.0f;
    bool has_cylinder_ = false;
    bool has_locked_target_ = false;
    bool pipeline_done_ = false;
    bool use_previous_normal_ = false;
    bool generate_path_ = false;

    // ROS: Subscriptions/ publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_, processed_pub_, cluster_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr normals_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
    rclcpp::TimerBase::SharedPtr collection_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rim_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cylinder_marker_pub_;
    //rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // For cylinder rim and axis visualisation
    Eigen::Vector3f last_cylinder_axis_point_;
    Eigen::Vector3f last_cylinder_axis_dir_;

    // State from the most recent frame, used by onClickedPoint()
    pcl::PointCloud<PointT>::Ptr last_cloud_;
    pcl::PointCloud<pcl::Normal>::Ptr last_normals_;
    std::vector<pcl::PointIndices> last_clusters_;
    std_msgs::msg::Header last_header_;
    sensor_msgs::msg::PointCloud2 last_processed_msg_;
    visualization_msgs::msg::MarkerArray last_normals_msg_;


    // a timer to publish processed result (global map) later (different from collection timer)
    rclcpp::TimerBase::SharedPtr keepalive_timer_;

    // for locked target
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PoseStamped locked_target_;

    // for multiple normals
    std::vector<geometry_msgs::msg::PoseStamped> selected_normals_;
    double normal_merge_radius_;          // declare_parameter("normal_merge_radius", 0.03)
    std::string normals_save_path_;       // HOME + "/vision_ws/selected_normals.yaml"
    // Locked normal targets
    std::vector<geometry_msgs::msg::PoseStamped> locked_targets_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr targets_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr targets_markers_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_selection_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_path_srv_;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_, undo_srv_, save_srv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPipeline>()); // executor: single-threaded
    rclcpp::shutdown();
    return 0;
}