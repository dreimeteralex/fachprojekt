#include <chrono>
#include <cstddef>
#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class WFNode : public rclcpp::Node 
{
public:
    WFNode() : Node("wallfollowing_node"), velocity_x_(0.0)
    {
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot1/scan", 
            10, 
            std::bind(&WFNode::laserCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot1/odom",
            10,
            std::bind(&WFNode::odomCallback, this, std::placeholders::_1));

        // WICHTIG: Topic für Multiplexer
        wall_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/robot1/cmd_vel/wall_follower", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&WFNode::timer_callback, this));

        last_test_log_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Wallfollowing Node started!");
    }

private:

    // ============================================================
    // DEBUG-TESTS (throttled auf 0.5 Sekunden)
    // ============================================================
    void debug_tests(double sen_A, double sen_B, double steer, double lin_vel, double distance)
    {
        rclcpp::Time now = this->now();

        // nur alle 0.5 Sekunden ausgeben
        if ((now - last_test_log_time_).seconds() < 0.5) {
            return;
        }
        last_test_log_time_ = now;

        if (!std::isfinite(sen_A) || !std::isfinite(sen_B)) {
            RCLCPP_WARN(this->get_logger(), "Test FAILED: Invalid sensor readings A or B");
        }

        if (!std::isfinite(steer)) {
            RCLCPP_ERROR(this->get_logger(), "Test FAILED: Steer is NaN or INF");
        }

        if (std::abs(steer) > 3.0) {
            RCLCPP_WARN(this->get_logger(), "Test WARNING: Steer value too high = %f", steer);
        }

        if (lin_vel < 0.0) {
            RCLCPP_ERROR(this->get_logger(), "Test FAILED: Negative linear velocity!");
        }

        RCLCPP_INFO(this->get_logger(),
            "TEST OK | A=%.2f B=%.2f steer=%.2f vel=%.2f distance=%.2f",
            sen_A, sen_B, steer, lin_vel, distance
        );
    }

    // ============================================================
    // WALLFOLLOWING
    // ============================================================
    void wallfollowing(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    if (!scan || scan->ranges.empty()) {
        RCLCPP_WARN(this->get_logger(), "Leerer oder ungültiger Scan!");
        return;
    }

    // Feste Werte für die Winkel von Beam A und Beam B (70° und 90° rechts)
    size_t val_A = 260; // ~70° rechts-vorne
    size_t val_B = 180; // ~90° rechts seitlich

    // Feste Werte für die Entfernungen an den Indizes
    double sen_A = scan->ranges[val_A];
    double sen_B = scan->ranges[val_B];

    // Überprüfen, ob die Werte gültig sind
    if (!std::isfinite(sen_A) || !std::isfinite(sen_B)) {
        RCLCPP_WARN(this->get_logger(), "Ungültiger Sensorwert!");
        return;
    }

    // Winkel zwischen Beam A und Beam B (20°)
    double theta = 20.0 * M_PI / 180.0;

    // Winkel Alpha (relativer Winkel zwischen den beiden Sensoren)
    double alpha = atan((sen_A * cos(theta) - sen_B) / (sen_A * sin(theta)));
    double degree_alpha = alpha * (180.0 / M_PI);  // nur noch für Logging / Debug

    // Vorgesehene Distanz zur Wand und der Lookahead-Abstand
    double desired_dis = 0.6;
    double lookahead   = 0.4;

    // Aktueller Abstand zur Wand
    double distance = sen_B * cos(alpha);

    // Zukünftiger Abstand zur Wand
    double D_t1 = distance + lookahead * sin(alpha);

    // Fehler: positiv -> zu nah an der Wand, negativ -> zu weit weg
    double error = desired_dis - D_t1;

    // --- NEU: steer nur über error, mit einfachem P-Anteil ---
    // error > 0  -> steer > 0  -> nach links lenken
    // error < 0  -> steer < 0  -> nach rechts lenken
    const double Kp = 1.0;  // kannst du tunen
    double steer = Kp * error;

    // Optional: Lenkung begrenzen, damit es nicht zu aggressiv wird
    const double max_steer = 0.6; // je nach Fahrzeug/SIM anpassen
    if (steer > max_steer)  steer = max_steer;
    if (steer < -max_steer) steer = -max_steer;

    // Nachricht erstellen und den Steuerbefehl an den Roboter senden
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    msg.twist.angular.z = steer;

    // Geschwindigkeit aktuell noch nach Winkel-Info (kannst du später auch auf |error| umstellen)
    if (std::fabs(degree_alpha) <= 10.0) {
        msg.twist.linear.x = 3;
    } 
    else if (std::fabs(degree_alpha) <= 20.0) {
        msg.twist.linear.x = 2.3;
    } 
    else {
        msg.twist.linear.x = 1.5;
    }

    // Tests ausführen (alle 0.5s)
    debug_tests(sen_A, sen_B, steer, msg.twist.linear.x, distance);

    // Publizieren an den Multiplexer
    wall_pub->publish(msg);
}





    // ============================================================
    // CALLBACKS
    // ============================================================
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        velocity_x_ = msg->twist.twist.linear.x;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;
        wallfollowing(last_scan_);
    }

    void timer_callback()
    {
        // unbenutzt
    }

    // ============================================================
    // MEMBER
    // ============================================================
    double velocity_x_;
    rclcpp::Time last_test_log_time_;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr wall_pub;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================
// MAIN
// ============================================================
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WFNode>());
    rclcpp::shutdown();
    return 0;
}
