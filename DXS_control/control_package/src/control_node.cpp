#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "control_package/lon_control.hpp" // LonController 헤더 파일 포함
#include "control_package/lat_control.hpp" // CombinedController 포함

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <fstream>
#include <sstream>

// 두 점 사이의 거리 계산 함수
double distance(const Point& p1, const Point& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

class ControlNode : public rclcpp::Node
{
public:
    ControlNode()
        : Node("control_node"),
        lon_controller_(std::make_shared<LonController>()),
        combined_controller_(std::make_shared<CombinedController>()),
        speed_(0.0),
        heading_(0.0), // 초기 헤딩값 0으로 설정
        gps_heading_(0.0), // 초기 GPS 헤딩값 설정
        previous_gps_heading_(0.0), // 초기 이전 GPS 헤딩값 설정
        current_odom_{ 0.0, 0.0 } // current_odom_을 (0, 0)으로 고정
    {
        steering_publisher_ = this->create_publisher<std_msgs::msg::Float64>("angular_cmd", 10);
        lon_control_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("linear_cmd", 10);
        relative_heading_publisher_ = this->create_publisher<std_msgs::msg::Float64>("relative_heading", 10); // relative_heading 퍼블리셔 추가

        control_path_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/midpoints_float", 10, std::bind(&ControlNode::midpointsFloatCallback, this, std::placeholders::_1));

        current_vel_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/current_vel", 10, std::bind(&ControlNode::currentVelCallback, this, std::placeholders::_1));

        // /current_heading 토픽 구독
        current_heading_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/current_heading", 10, std::bind(&ControlNode::currentHeadingCallback, this, std::placeholders::_1));

        // /ekf_heading 토픽 구독 추가
        ekf_heading_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/ekf_heading", 10, std::bind(&ControlNode::ekfHeadingCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ControlNode::publish_messages, this));
    }

private:
    void publish_messages()
    {
        if (path_.empty() || path_.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "곡률 계산을 위한 경로가 비어있거나 너무 짧습니다. 발행을 건너뜁니다.");
            return;
        }

        double curvature = calculateCurvatureByAngle(path_);
        lon_controller_->setCurvature(curvature);

        float calculated_speed = lon_controller_->targetSpeed();

        combined_controller_->setCombinedData(speed_, current_odom_, path_);

        // 조향 각도 계산 시 speed_를 전달
        double combined_steering_rad = combined_controller_->calcCombinedSteer();
        double combined_steering_deg = combined_steering_rad * (180.0 / M_PI);

        auto steering_msg = std_msgs::msg::Float64();
        steering_msg.data = combined_steering_deg;
        steering_publisher_->publish(steering_msg);

        auto lon_control_msg = std_msgs::msg::UInt8();
        lon_control_msg.data = static_cast<uint8_t>(calculated_speed);
        lon_control_publisher_->publish(lon_control_msg);

        // GPS 상대 헤딩 퍼블리시 조건 추가
        auto relative_heading_msg = std_msgs::msg::Float64();
        if (speed_ >= 0.1) {
            relative_heading_msg.data = gps_heading_ - previous_gps_heading_;
        } else {
            relative_heading_msg.data = 0.0;  // 속도가 0.1보다 작으면 0으로 설정
        }
        relative_heading_publisher_->publish(relative_heading_msg);

        // 이전 GPS 헤딩을 현재 헤딩으로 업데이트
        previous_gps_heading_ = gps_heading_;

        RCLCPP_INFO(this->get_logger(), "steer: %.2f degrees", steering_msg.data);
        RCLCPP_INFO(this->get_logger(), "lon: %u", lon_control_msg.data);
        RCLCPP_INFO(this->get_logger(), "curvature: %.4f", curvature);
        RCLCPP_INFO(this->get_logger(), "GPS heading: %.2f", gps_heading_);
        RCLCPP_INFO(this->get_logger(), "Relative heading: %.2f", relative_heading_msg.data);
    }

    double calculateCurvatureByAngle(const std::vector<Point>& path)
    {
        if (path.size() < 3)
            return 0.0;

        double total_curvature = 0.0;
        int count = 0;

        for (size_t i = 1; i < path.size() - 1; ++i) {
            double dx1 = path[i].x - path[i - 1].x;
            double dy1 = path[i].y - path[i - 1].y;
            double theta1 = atan2(dy1, dx1);

            double dx2 = path[i + 1].x - path[i].x;
            double dy2 = path[i + 1].y - path[i].y;
            double theta2 = atan2(dy2, dx2);

            double dtheta = std::abs(normalize_angle(theta2 - theta1));

            double ds = sqrt(dx1 * dx1 + dy1 * dy1);

            if (ds > 0.0) {
                double curvature = dtheta / ds;
                total_curvature += curvature;
                ++count;
            }
        }

        if (count > 0)
            return total_curvature / count;
        else
            return 0.0;
    }

    // /midpoints_float 토픽 콜백 함수 (상대 좌표 그대로 사용)
    void midpointsFloatCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        path_.clear();

        for (size_t i = 0; i < msg->data.size(); i += 3) {
            double x = msg->data[i + 1];  // x 좌표 (상대 좌표)
            double y = msg->data[i + 2];  // y 좌표 (상대 좌표)
            path_.emplace_back(Point{ x, y }); // 상대 좌표 그대로 추가
        }
    }

    // /current_vel 토픽 콜백 함수 (속도 업데이트)
    void currentVelCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        speed_ = msg->data;
    }

    // /current_heading 토픽 콜백 함수 (헤딩 업데이트)
    void currentHeadingCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        heading_ = msg->data;
    }

    // /ekf_heading 토픽 콜백 함수 (GPS 헤딩 업데이트)
    void ekfHeadingCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        gps_heading_ = msg->data;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr lon_control_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr relative_heading_publisher_; // relative_heading 퍼블리셔

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr control_path_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_vel_subscription_; // /current_vel 구독 추가
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_heading_subscription_; // /current_heading 구독 추가
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ekf_heading_subscription_; // /ekf_heading 구독 추가
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<LonController> lon_controller_;
    std::shared_ptr<CombinedController> combined_controller_;

    std::vector<Point> path_;
    Point current_odom_; // 고정된 초기 값 (0,0)
    double speed_;
    double heading_; // 현재 헤딩 값
    double gps_heading_; // GPS 헤딩 값
    double previous_gps_heading_; // 이전 GPS 헤딩 값
};

// 메인 함수
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}

