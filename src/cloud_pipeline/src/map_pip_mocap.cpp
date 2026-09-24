// Map pipeline to work with mocap. 
// Using since 18 August 2026
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


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

class MapPipeline : public rclcpp::Node
{
public:
    MapPipeline() : Node("cloud_pipeline")
    {
        // TOPICS
        //cloud_topic_       = declare_parameter<std::string>("cloud_topic", "/dlio/odom_node/pointcloud/deskewed");
        cloud_topic_       = declare_parameter<std::string>("cloud_topic", "/livox/lidar");
        clicked_topic_     = declare_parameter<std::string>("clicked_topic", "/clicked_point");
        global_map_topic_  = declare_parameter<std::string>("global_map_topic", "/global_map");
        processed_topic_   = declare_parameter<std::string>("processed_topic", "/processed/map");
        normals_topic_     = declare_parameter<std::string>("normals_topic", "/processed/normals");
        clusters_topic_    = declare_parameter<std::string>("clusters_topic", "/processed/clusters");
        target_topic_      = declare_parameter<std::string>("locked_target", "/processed/locked_target");
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
        save_path_ = declare_parameter<std::string>("save_path",std::string(std::getenv("HOME")) + "/vision_ws/global_map.pcd");       
        mesh_path_ = declare_parameter<std::string>("mesh_path",""); 
        normal_file_path_ = declare_parameter<std::string>("normal_file_path",""); 
        
        // ----------------
        // OPERATIONS
        collection_duration_sec_ = declare_parameter<double>("collection_duration_sec", 15.0);
        use_previous_normal_     = declare_parameter<bool>("use_previous_normal", false);

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
        max_ransac_iterations_ = declare_parameter<int>("max_ransac_iterations", 1000);
        min_ransac_radius_            = declare_parameter<double>("min_radius", 0.005);
        max_ransac_radius_            = declare_parameter<double>("max_radius", 0.15);
        ransac_probability_    = declare_parameter<double>("ransac_probability", 0.95);

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

        target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            target_topic_, rclcpp::QoS(1).transient_local());

        rim_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            rim_topic_, rclcpp::QoS(1).transient_local());

        cylinder_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            cylinder_marker_topic_, rclcpp::QoS(1).transient_local());

        mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            mesh_topic_, rclcpp::QoS(1).transient_local());
        
        
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

        RCLCPP_INFO(get_logger(),
            "cloud_pipeline: collecting from '%s' for %.1f seconds...",
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
    // ---- runs every incoming scan, only while collecting_ is true ----
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
    

        // Print accumulated map size, every 2 seconds to prevent terminal overloading. 
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Collecting... accumulated map: %zu points.", global_map_->size());

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
                "User selected previous normal.");

            publishPreviousNormal();
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

    bool loadPreviousNormal(
        Eigen::Vector3d& point,
        Eigen::Quaterniond& orientation)
    {
        const std::string path = this->get_parameter("normal_file_path").as_string();

        std::ifstream file(path);

        if (!file.is_open())
        {
            RCLCPP_WARN(
                get_logger(),
                "Could not open previous normal file: %s",
                path.c_str());

            return false;
        }

        std::string line;

        bool in_position = false;
        bool in_orientation = false;

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;

        while (std::getline(file, line))
        {
            // ---------------------------------------------------------
            // Detect sections
            // ---------------------------------------------------------

            if (line.find("position:") != std::string::npos)
            {
                in_position = true;
                in_orientation = false;
                continue;
            }

            if (line.find("orientation:") != std::string::npos)
            {
                in_position = false;
                in_orientation = true;
                continue;
            }

            // ---------------------------------------------------------
            // Read position
            // ---------------------------------------------------------

            if (in_position)
            {
                if (line.find("x:") != std::string::npos)
                    x = std::stod(
                        line.substr(line.find(":") + 1));

                else if (line.find("y:") != std::string::npos)
                    y = std::stod(
                        line.substr(line.find(":") + 1));

                else if (line.find("z:") != std::string::npos)
                    z = std::stod(
                        line.substr(line.find(":") + 1));
            }

            // ---------------------------------------------------------
            // Read orientation
            // ---------------------------------------------------------

            if (in_orientation)
            {
                if (line.find("x:") != std::string::npos)
                    qx = std::stod(
                        line.substr(line.find(":") + 1));

                else if (line.find("y:") != std::string::npos)
                    qy = std::stod(
                        line.substr(line.find(":") + 1));

                else if (line.find("z:") != std::string::npos)
                    qz = std::stod(
                        line.substr(line.find(":") + 1));

                else if (line.find("w:") != std::string::npos)
                    qw = std::stod(
                        line.substr(line.find(":") + 1));
            }
        }

        file.close();

        // ---------------------------------------------------------
        // Construct Eigen objects
        // ---------------------------------------------------------

        point = Eigen::Vector3d(x, y, z);

        orientation = Eigen::Quaterniond(
            qw,
            qx,
            qy,
            qz);

        // ---------------------------------------------------------
        // Validate
        // ---------------------------------------------------------

        if (!std::isfinite(orientation.x()) ||
            !std::isfinite(orientation.y()) ||
            !std::isfinite(orientation.z()) ||
            !std::isfinite(orientation.w()))
        {
            RCLCPP_WARN(
                get_logger(),
                "Previous orientation contains invalid values.");

            return false;
        }

        if (orientation.norm() < 1e-6)
        {
            RCLCPP_WARN(
                get_logger(),
                "Previous orientation has near-zero magnitude.");

            return false;
        }

        orientation.normalize();

        // ---------------------------------------------------------
        // Print
        // ---------------------------------------------------------

        RCLCPP_INFO(
            get_logger(),
            "Loaded previous target:");

        RCLCPP_INFO(
            get_logger(),
            "  Point:       (%.3f, %.3f, %.3f)",
            point.x(),
            point.y(),
            point.z());

        RCLCPP_INFO(
            get_logger(),
            "  Orientation: (%.3f, %.3f, %.3f, %.3f)",
            orientation.x(),
            orientation.y(),
            orientation.z(),
            orientation.w());

        return true;
    }

    void publishPreviousNormal()
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

        // Load the saved normal
        if (!loadPreviousNormal(position, orientation))
        {
            RCLCPP_WARN(
                get_logger(),
                "Could not load previous normal.");
            return;
        }

        geometry_msgs::msg::PoseStamped pose;

        pose.header.frame_id = frame_id_;
        pose.header.stamp = get_clock()->now();

        // Position where the previous normal was selected
        pose.pose.position.x = position.x();
        pose.pose.position.y = position.y();
        pose.pose.position.z = position.z();

        // Orientation representing normal direction
        pose.pose.orientation.x = orientation.x();
        pose.pose.orientation.y = orientation.y();
        pose.pose.orientation.z = orientation.z();
        pose.pose.orientation.w = orientation.w();

        // Publish
        target_pub_->publish(pose);

        RCLCPP_INFO(
            get_logger(),
            "Published previous normal pose:");
            
        RCLCPP_INFO(
            get_logger(),
            "  Position: (%.3f, %.3f, %.3f)",
            position.x(),
            position.y(),
            position.z());

        RCLCPP_INFO(
            get_logger(),
            "  Normal:   (%.3f, %.3f, %.3f, %.3f)",
            orientation.x(),
            orientation.y(),
            orientation.z(),
            orientation.w());
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
        int stride = std::max(1, marker_stride_);
        int id = 0;

        for (size_t i = 0; i < cloud->size(); i += static_cast<size_t>(stride))
        {
            const auto &p = cloud->points[i];
            const auto &n = normals->points[i];
            if (!std::isfinite(n.normal_x) || !std::isfinite(n.normal_y) || !std::isfinite(n.normal_z))
                continue;

            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = frame_id_;
            arrow.header.stamp = stamp;
            arrow.ns = "map_normals";
            arrow.id = id++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point p0, p1;
            p0.x = p.x; p0.y = p.y; p0.z = p.z;
            p1.x = p.x + n.normal_x * marker_scale_;
            p1.y = p.y + n.normal_y * marker_scale_;
            p1.z = p.z + n.normal_z * marker_scale_;
            arrow.points.push_back(p0);
            arrow.points.push_back(p1);

            // Shaft diameter, head diameter, head length — tune these smaller
            // than the defaults (which are sized for scale.x ~ 1.0 lines).
            arrow.scale.x = 0.002;  // shaft diameter
            arrow.scale.y = 0.004;  // head diameter
            arrow.scale.z = 0.006;  // head length

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

    void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!pipeline_done_ || collecting_)// ignore if collection and pipeline not done
        {
            RCLCPP_WARN(get_logger(), "Pipeline not ready for clicks.");
            return;
        }; 

            // Process clicked point
        if (last_clusters_.empty() || !last_cloud_ || !last_normals_) {
            RCLCPP_WARN(get_logger(), "No cluster data yet, ignoring click");
            return;
        }

        // 1.5 Find the nearest point of the click

        double best_dist = std::numeric_limits<double>::max();
        int best_idx = -1;

        for (size_t i = 0; i < last_cloud_->size(); ++i)
        {
            double dx = last_cloud_->points[i].x - msg->point.x;
            double dy = last_cloud_->points[i].y - msg->point.y;
            double dz = last_cloud_->points[i].z - msg->point.z;

            double d = dx*dx + dy*dy + dz*dz;

            if (d < best_dist)
            {
                best_dist = d;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx < 0)
            return;

        // 2. Get the exact point and its corresponding normal
        // ---------------------------------------------------------
        const auto& clicked_point = last_cloud_->points[best_idx];
        const auto& normal = last_normals_->points[best_idx];

        if (!std::isfinite(normal.normal_x) ||
            !std::isfinite(normal.normal_y) ||
            !std::isfinite(normal.normal_z))
        {
            RCLCPP_WARN(get_logger(),
                        "Clicked point has invalid normal, ignoring click");
            return;
        }

        Eigen::Vector3f normal_vec =
            normal.getNormalVector3fMap();

        normal_vec.normalize();

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
            std::sqrt(best_dist));

        // ---------------------------------------------------------
        // 3. Create target at the clicked/nearest map point
        // ---------------------------------------------------------
        geometry_msgs::msg::PoseStamped target;

        target.header = last_header_;

        target.pose.position.x = clicked_point.x;
        target.pose.position.y = clicked_point.y;
        target.pose.position.z = clicked_point.z;

        // ---------------------------------------------------------
        // 4. Convert normal into orientation
        //    so the pose's +X axis points along the normal
        // ---------------------------------------------------------
        Eigen::Vector3f x_axis(1.0f, 0.0f, 0.0f);

        Eigen::Quaternionf q =
            Eigen::Quaternionf::FromTwoVectors(x_axis, normal_vec);

        target.pose.orientation.x = q.x();
        target.pose.orientation.y = q.y();
        target.pose.orientation.z = q.z();
        target.pose.orientation.w = q.w();

        // ---------------------------------------------------------
        // 5. Transform into stable/world frame
        // ---------------------------------------------------------
        try
        {
            geometry_msgs::msg::PoseStamped target_in_world;

            tf_buffer_->transform(
                target,
                target_in_world,
                frame_id_,
                tf2::durationFromSec(0.0));

            locked_target_ = target_in_world;
            has_locked_target_ = true;
            saveLockedTarget();

            RCLCPP_INFO(
                get_logger(),
                "Locked target in 'world' frame at "
                "(%.3f, %.3f, %.3f)",
                target_in_world.pose.position.x,
                target_in_world.pose.position.y,
                target_in_world.pose.position.z);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(
                get_logger(),
                "Could not transform target into world frame: %s",
                ex.what());
        }

        // ---------------------------------------------------------
        // 6. Publish
        // ---------------------------------------------------------
        target_pub_->publish(target);
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

        const std::string path = this->get_parameter("normal_file_path").as_string();

        std::ofstream file(path);

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
            "Saved locked target to locked_target.yaml");
    }

    // params
    std::string cloud_topic_, global_map_topic_, processed_topic_, normals_topic_, frame_id_, save_path_, mesh_path_, normal_file_path_,
                clicked_topic_, clusters_topic_, target_topic_, rim_topic_, cylinder_marker_topic_, mesh_topic_;
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPipeline>()); // executor: single-threaded
    rclcpp::shutdown();
    return 0;
}