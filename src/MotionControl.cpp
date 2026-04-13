#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry and laser scans
        // add code here
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));
        
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/tiago_base/Hokuyo_URG_04LX_UG01", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));
        
        // Publisher for robot control
        // add code here
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Client for path planning
        // add code here
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

        // Action server
        // add code here
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this, "/go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server
        // add code here
        while (!plan_client_->wait_for_service(std::chrono::seconds(2))) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(get_logger(), "Waiting for /plan_path service...");
        }
    }

void MotionControlNode::checkCollision() {
    // add code here
    if (laser_scan_.ranges.empty()) return; // Pokud ještě nepřišla data z lidaru

    double emergency_threshold = 0.45; // 30 centimetrů od překážky
    bool collision_imminent = false;

    int num_rays = laser_scan_.ranges.size();
    int center_ray = num_rays / 2; // Středový paprsek mířící přesně dopředu
    int cone_width = 180; // Kolik paprsků na každou stranu zkontrolujeme (šířka kuželu)

    // Výpočet začátku a konce hledání (s ochranou proti přetečení pole)
    int start_idx = std::max(0, center_ray - cone_width);
    int end_idx = std::min(num_rays, center_ray + cone_width);

    // Kontrolujeme jen výseč před robotem
    for (int i = start_idx; i < end_idx; i++) {
        float r = laser_scan_.ranges[i];
        
        // Stále ignorujeme NaN a hodnoty blížící se 0.0
        if (!std::isnan(r) && r > 0.05 && r < emergency_threshold) {
            collision_imminent = true;
            break; 
        }
    }

    if (collision_imminent && goal_handle_ && goal_handle_->is_active()) {
        RCLCPP_WARN(this->get_logger(), "POZOR! Překážka příliš blízko! Nouzové zastavení.");
        
        // Zastavíme motory
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
        
        // Zrušíme navigační akci (abort)
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
    }
    // ********
    // * Help *
    // ********
    /*
    if (laser_scan_.ranges[i] < thresh) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
    }
    */
}

void MotionControlNode::updateTwist() {
    // add code here
    // 1. Zkontrolujeme, jestli vůbec máme kam jet
    if (!goal_handle_ || !goal_handle_->is_active() || path_.poses.empty()) {
        return;
    }

    // Získání aktuální pozice robota (rx, ry)
    double rx = current_pose_.pose.position.x;
    double ry = current_pose_.pose.position.y;

    // Převod kvaternionu z orientace na Eulerovy úhly, abychom zjistili YAW (kam robot kouká)
    tf2::Quaternion q;
    tf2::fromMsg(current_pose_.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 2. Najdeme nejbližší bod na trase k robotovi
    int closest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < path_.poses.size(); i++) {
        double dist = std::hypot(path_.poses[i].pose.position.x - rx, path_.poses[i].pose.position.y - ry);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // 3. Najdeme "mrkev" (Cílový bod) kousek před námi (Lookahead distance)
    double lookahead_dist = 0.5; // Díváme se půl metru dopředu
    int target_idx = closest_idx;
    
    for (size_t i = closest_idx; i < path_.poses.size(); i++) {
        double dist = std::hypot(path_.poses[i].pose.position.x - rx, path_.poses[i].pose.position.y - ry);
        if (dist > lookahead_dist) {
            target_idx = i;
            break;
        }
    }

    double target_x = path_.poses[target_idx].pose.position.x;
    double target_y = path_.poses[target_idx].pose.position.y;

    // 4. Výpočet odchylky natočení (Heading error)
    // Nejdřív spočítáme úhel z naší pozice k cílovému bodu v mapě
    double angle_to_target = std::atan2(target_y - ry, target_x - rx);
    
    // Rozdíl mezi tím, kam se chceme dívat, a kam aktuálně koukáme
    double heading_error = angle_to_target - yaw;

    // Normalizace úhlu, aby robot netočil o 300 stupňů doprava místo 60 doleva
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

    // 5. Řízení rychlostí
    double Kp_angular = 1.5; // Zesílení (Jak agresivně zatáčí)
    double angular_vel = Kp_angular * heading_error;

    // Dopředná rychlost
    // Podle zadání max kola točí ~10 rad/s * 0.0985 m = cca 1 m/s. Raději začneme na velmi bezpečné rychlosti
    double linear_vel = 0.2; // 20 cm za sekundu

    // Pokud je zatáčka příliš ostrá (chyba je větší než cca 30 stupňů), robot téměř zastaví a otočí se na místě
    if (std::abs(heading_error) > 0.5) {
        linear_vel = 0.05; 
    }

    // 6. Saturace (Ořezání extrémů)
    if (linear_vel > 0.4) linear_vel = 0.4;
    if (angular_vel > 1.0) angular_vel = 1.0;
    if (angular_vel < -1.0) angular_vel = -1.0;

    // 7. Odeslání rychlostí do simulátoru
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_vel;
    cmd.angular.z = angular_vel;
    twist_publisher_->publish(cmd);

    // ********
    // * Help *
    // ********
    /*
    geometry_msgs::msg::Twist twist;
    twist.angular.z = P * xte;
    twist.linear.x = v_max;

    twist_publisher_->publish(twist);
    */
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    // add code here
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid; // Umlčení warningu z kompilátoru
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    // ********
    // * Help *
    // ********
    /*
    (void)uuid;
    ...
    return ...;
    */
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle; // Umlčení warningu z kompilátoru
    return rclcpp_action::CancelResponse::ACCEPT;

    // ********
    // * Help *
    // ********
    /*
    (void)goal_handle;
    ...
    return ...;
    */
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here
    goal_handle_ = goal_handle;
    goal_pose_ = goal_handle->get_goal()->pose;

    RCLCPP_INFO(this->get_logger(), "Goal accepted, requesting path from planner...");

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_; // <-- TADY VYUŽÍVÁME AKTUÁLNÍ POZICI ROBOTA!
    request->goal = goal_pose_;
    request->tolerance = 0.0;
    // ********
    // * Help *
    // ********
    /*
    ...
    */
    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    // add code here
    rclcpp::Rate loop_rate(5.0); // 5 Hz je pro kontrolu navigace tak akorát

    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

    while (rclcpp::ok()) {
        // Kontrola, zda uživatel (nebo jiný uzel) nepožádal o zrušení akce
        if (!goal_handle_->is_active()) return;
        if (goal_handle_->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Navigace byla zrušena.");
            
            // Zastavíme robota
            geometry_msgs::msg::Twist stop_msg;
            twist_publisher_->publish(stop_msg);
            
            goal_handle_->canceled(result);
            return;
        }

        // Tady bychom správně měli počítat zbývající vzdálenost a plnit 'feedback'
        // Pro zjednodušení si zatím jen posíláme prázdný feedback
        goal_handle_->publish_feedback(feedback);

        // Zjištění, zda jsme v cíli
        // (Vypočítáme vzdálenost mezi aktuální pozicí a cílem)
        double dx = current_pose_.pose.position.x - goal_pose_.pose.position.x;
        double dy = current_pose_.pose.position.y - goal_pose_.pose.position.y;
        double distance_to_goal = std::hypot(dx, dy);

        if (distance_to_goal < 0.2) { // Pokud jsme blíž než 20 cm, bereme to jako úspěch
            RCLCPP_INFO(this->get_logger(), "Cíl úspěšně dosažen!");
            
            // Zastavíme robota
            geometry_msgs::msg::Twist stop_msg;
            twist_publisher_->publish(stop_msg);
            if (goal_handle_->is_active()) {
                goal_handle_->succeed(result);
            }
            return;
        }

        loop_rate.sleep();
    }

    // ********
    // * Help *
    // ********
    /*
    rclcpp::Rate loop_rate(1.0); // 1 Hz

    while (rclcpp::ok()) {

        if (goal_handle_->is_canceling()) {
            ...
            return;
        }

        ...

        goal_handle_->publish_feedback(feedback);

        loop_rate.sleep();
    }
    */
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    // add code here
    auto response = future.get();
    
    // Zkontrolujeme, zda jsme dostali platnou trasu (delší než 0 bodů)
    if (response && response->plan.poses.size() > 0) {
        path_ = response->plan; // Uložíme si ji do naší proměnné
        RCLCPP_INFO(this->get_logger(), "Trasa přijata! Počet bodů: %zu. Začínám navigovat.", path_.poses.size());
        
        // Spuštění akce a prováděcí smyčky v novém vlákně (podle tipu ze zadání)
        // goal_handle_->execute();
        std::thread(&MotionControlNode::execute, this).detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Plánovač nenašel cestu k cíli!");
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result); // Ukončíme akci s chybou
    }
    // ********
    // * Help *
    // ********
    /*
    if (response && response->plan.poses.size() > 0) {
        goal_handle_->execute();
        std::thread(&MotionControlNode::execute, this).detach();
    }
    */
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    // add code here
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;

    // ********
    // * Help *
    // ********
    /*
    */
    // checkCollision();
    updateTwist();
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    // add code here
    laser_scan_ = msg;
}
