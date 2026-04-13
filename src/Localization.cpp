#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    // add code here

    odometry_.pose.pose.position.x = -0.5;
    odometry_.pose.pose.position.y = 0.0;
    odometry_.pose.pose.position.z = 0.0;

    // Subscriber for joint_states
    // add code here
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
    );

    // Publisher for odometry
    // add code here
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // add code here
    // RCLCPP_INFO(this->get_logger(), "Přijata data z enkodérů! Počet kloubů: %zu", msg.name.size());

    // ********
    // * Help *
    // ********
    auto current_time = this->get_clock()->now();

    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    
    updateOdometry(msg.velocity[1], msg.velocity[0], dt);
    publishOdometry();
    publishTransform();
    /*
    */
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // add code here

    // ********
    // * Help *
    // ********
    double linear =  (robot_config::WHEEL_RADIUS/2)*(right_wheel_vel+left_wheel_vel);
    double angular = (robot_config::WHEEL_RADIUS/(2*robot_config::HALF_DISTANCE_BETWEEN_WHEELS))*(right_wheel_vel-left_wheel_vel);  //robot_config::HALF_DISTANCE_BETWEEN_WHEELS
    
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    odometry_.pose.pose.position.x += linear * std::cos(theta)* dt;
    odometry_.pose.pose.position.y += linear * std::sin(theta)* dt;
    theta += angular *dt;
    
    theta = std::atan2(std::sin(theta), std::cos(theta));
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta); // byla tu 0
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    // 6. Uložení aktuálních rychlostí do zprávy
    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = angular;
    /*
    */
}

void LocalizationNode::publishOdometry() {
    // add code here
    odometry_.header.stamp = this->get_clock()->now();
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";

    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    // add code here
    geometry_msgs::msg::TransformStamped t;
    
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    // Pozice x, y získáme z naší již spočítané odometrie
    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0;

    // Rotace (přesuneme ten stejný kvaternion z odometrie do transformace)
    t.transform.rotation = odometry_.pose.pose.orientation;

    // ********
    // * Help *
    // ********
    tf_broadcaster_->sendTransform(t);
}
