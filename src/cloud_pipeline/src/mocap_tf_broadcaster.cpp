// mocap_tf_broadcaster.cpp
//
// Republishes an OptiTrack rigid body (from optitrack_multiplexer_ros2)
// as a dynamic TF, purely so it can be watched live in RViz -- e.g. to
// see the drone's mocap-tracked pose/odometry independent of whether
// the map pipeline is running.
//
// map_pip_optitrack.cpp does NOT need this node: it consumes
// RigidBodyStamped directly and computes
//     T_world_lidar = T_world_body * T_body_lidar
// itself (lerp/slerp-interpolated to each scan's exact timestamp), and
// never calls tf_buffer_->lookupTransform() for it. This node exists
// only for visualisation, not as a dependency of the pipeline.
//
// Message shape confirmed against map_pip_optitrack.cpp's working
// mocapCallback() (optitrack_multiplexer_ros2_msgs/msg/RigidBodyStamped):
//   msg->stamp                          builtin_interfaces/Time
//   msg->rigid_body.name                string
//   msg->rigid_body.tracking_valid      bool
//   msg->rigid_body.mean_error          float
//   msg->rigid_body.pose.position.{x,y,z}
//   msg->rigid_body.pose.orientation.{q_w,q_x,q_y,q_z}   <- NOT geometry_msgs/Quaternion's w/x/y/z
//
// Subscribes:
//   <pose_topic>  (default "/optitrack_multiplexer_node/rigid_body/<mocap_rigid_body>",
//                  same convention map_pip_optitrack.cpp uses)
//
// Broadcasts (dynamic TF):
//   <world_frame> -> <body_frame>, stamped with the message's own
//   stamp (not now()), so anything else consuming this TF (e.g. RViz's
//   own interpolation) sees it at the correct time rather than "whenever
//   this node happened to receive it".
//
// No buffering/interpolation here (unlike map_pip_optitrack.cpp) --
// a straight per-message republish is all RViz's TF display needs.
//
// Author: Thanh Tin Nguyen
// Email: ttn32@cam.ac.uk

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <optitrack_multiplexer_ros2_msgs/msg/rigid_body_stamped.hpp>

class MocapTfBroadcaster : public rclcpp::Node
{
public:
    MocapTfBroadcaster() : Node("mocap_tf_broadcaster")
    {
        mocap_rigid_body_ = declare_parameter<std::string>(
            "mocap_rigid_body", "drone");
        pose_topic_ = declare_parameter<std::string>("pose_topic", "");
        world_frame_ = declare_parameter<std::string>(
            "world_frame", "world");
        body_frame_ = declare_parameter<std::string>("body_frame", "");

        // Same defaulting convention as map_pip_optitrack.cpp, so the two
        // nodes agree on a topic/frame without needing every param typed
        // out in the launch script.
        if (pose_topic_.empty())
        {
            pose_topic_ =
                "/optitrack_multiplexer_node/rigid_body/" + mocap_rigid_body_;
        }
        if (body_frame_.empty())
        {
            body_frame_ = mocap_rigid_body_;
        }

        // Reject poses whose Motive mean marker error exceeds this (m).
        // Set <= 0 to disable the check. Same knob as map_pip_optitrack.cpp
        // for consistency, though a bad sample here only glitches a
        // visualisation, not the map.
        max_mean_error_ = declare_parameter<double>("max_mean_error", 0.0);

        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Reliable KeepLast(200), matching map_pip_optitrack.cpp's
        // subscription -- the multiplexer publishes at its default QoS
        // (reliable), so best-effort/SensorDataQoS here would silently
        // drop everything.
        pose_sub_ = create_subscription<
            optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>(
            pose_topic_, rclcpp::QoS(rclcpp::KeepLast(200)),
            std::bind(&MocapTfBroadcaster::poseCallback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "mocap_tf_broadcaster: subscribing '%s', broadcasting "
            "TF '%s' -> '%s'",
            pose_topic_.c_str(), world_frame_.c_str(), body_frame_.c_str());
    }

private:
    void poseCallback(
        const optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped::SharedPtr msg)
    {
        const auto &rb = msg->rigid_body;

        if (!rb.tracking_valid)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Rigid body '%s' reports tracking_valid=false -- "
                "occluded or out of view, skipping this sample.",
                rb.name.c_str());
            return;
        }

        if (max_mean_error_ > 0.0 &&
            static_cast<double>(rb.mean_error) > max_mean_error_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Rigid body '%s' mean error %.4f m exceeds %.4f m -- "
                "skipping this sample.",
                rb.name.c_str(),
                static_cast<double>(rb.mean_error), max_mean_error_);
            return;
        }

        geometry_msgs::msg::TransformStamped tf;

        // Use the message's own stamp, not now(), so whatever consumes
        // this TF sees it at the time the pose was actually measured.
        tf.header.stamp = msg->stamp;
        tf.header.frame_id = world_frame_;
        tf.child_frame_id = body_frame_;

        tf.transform.translation.x = rb.pose.position.x;
        tf.transform.translation.y = rb.pose.position.y;
        tf.transform.translation.z = rb.pose.position.z;

        // NOTE the field names: q_w/q_x/q_y/q_z, not geometry_msgs::Quaternion's
        // w/x/y/z -- this custom pose type does not assign-compatible with
        // tf.transform.rotation, so each component is copied individually.
        tf.transform.rotation.w = rb.pose.orientation.q_w;
        tf.transform.rotation.x = rb.pose.orientation.q_x;
        tf.transform.rotation.y = rb.pose.orientation.q_y;
        tf.transform.rotation.z = rb.pose.orientation.q_z;

        broadcaster_->sendTransform(tf);
    }

    std::string mocap_rigid_body_, pose_topic_, world_frame_, body_frame_;
    double max_mean_error_;
    rclcpp::Subscription<optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>::SharedPtr pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
