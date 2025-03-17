#ifndef LAT_CONTROLLERS_HPP
#define LAT_CONTROLLERS_HPP

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include "PID_control.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846 // M_PI가 정의되지 않은 경우, Pi 값 수동 정의
#endif

// Pure Pursuit 클래스 정의
// 차량이 주어진 경로를 따라가도록 순수 추종 제어를 구현하는 클래스
class PurePursuit {
private:
    float lookaheadDistance_; // 목표 지점까지의 추종 거리 (lookahead distance)
    float lookahead_gain_; // 추종 거리 계산을 위한 속도 이득 (gain)
    Point targetPoint_; // 계산된 목표 점 (lookahead point)

    Point current_odom_; // 현재 차량 위치 (Odometry 데이터)
    float speed_; // 차량의 현재 속도
    std::vector<Point> path_; // 주어진 경로 (path)
    float wheel_base_; // 차량의 바퀴 간 거리 (wheelbase)

    float ld_min_val_; // 추종 거리의 최소값
    float ld_max_val_; // 추종 거리의 최대값

    float PP_gain_; // Pure Pursuit 제어에서 조향 각도를 계산할 때 사용하는 이득 (gain)

public:
    // 생성자: 변수 초기화
    PurePursuit()
        : lookaheadDistance_(0.0),
          lookahead_gain_(1.0),
          targetPoint_(),
          current_odom_(0,0),
          speed_(0.0),
          wheel_base_(1.4),
          ld_min_val_(3.0),
          ld_max_val_(11.0),
          PP_gain_(1.0) {
    }

    // Pure Pursuit 제어기의 이득과 추종 거리를 설정하는 함수
    void setPPGain(double pp_gain, float lookahead) {
        PP_gain_ = pp_gain;
        lookahead_gain_ = lookahead;
    }

    // 추종 거리의 최소값과 최대값을 설정하는 함수
    void setPPLDThreshold(float min, float max) {
        ld_min_val_ = min;
        ld_max_val_ = max;
    }

    // Pure Pursuit 제어기에 필요한 데이터를 설정하는 함수
    void setPPData(float speed, const Point& current_odom, const std::vector<Point>& path) {
        speed_ = speed;
        current_odom_ = current_odom;
        path_ = path;
    }

    // 추종 거리 계산
    void calcLookaheadDistance() {
        lookaheadDistance_ = 5.0; // 속도에 기반한 추종 거리 계산
        lookaheadDistance_ = std::min(std::max(lookaheadDistance_, ld_min_val_), ld_max_val_); // 추종 거리의 값이 최소값과 최대값 사이에 있도록 제한
    }

    // 추종 거리 내의 목표 지점 (lookahead point) 계산
    Point calcLAPoint() {
        calcLookaheadDistance(); // 추종 거리 계산

        double minDistance = 1e9; // 임의의 큰 값으로 초기화 (최소 거리 추적용)
        Point closestPoint;

        for (const Point& p : path_) {
            // 목표 거리와 실제 경로 상 점 사이의 차이를 계산
            double dist_diff = std::abs(LADistance(p) - lookaheadDistance_);
            if (dist_diff < minDistance) {
                minDistance = dist_diff; // 차이가 최소인 점을 목표 지점으로 설정
                closestPoint = p;
            }
        }

        return closestPoint;
    }

    // 조향 각도 계산
    double calcPPSteer() {
        targetPoint_ = calcLAPoint(); // 추종 거리 내의 목표 지점 계산

        double dx = targetPoint_.x - current_odom_.x; // x 좌표 차이
        double dy = targetPoint_.y - current_odom_.y; // y 좌표 차이

        double angle_to_target = std::atan2(dy, dx); // 목표 지점까지의 각도
        double heading_error = angle_to_target; // 현재 방향과 목표 각도 간 오차
        heading_error = normalize_angle(heading_error); // 각도 오차를 -pi ~ pi로 정규화

        double steering_angle = std::atan2(2.0 * wheel_base_ * std::sin(heading_error), lookaheadDistance_); // 조향 각도 계산

        return steering_angle * PP_gain_; // 조향 각도에 이득을 적용하여 반환
    }

    // 현재 차량 위치에서 특정 점까지의 거리 계산
    double LADistance(const Point& p) {
        double dx = p.x - current_odom_.x;
        double dy = p.y - current_odom_.y;
        return std::sqrt(dx * dx + dy * dy); // 피타고라스 정리에 기반한 거리 계산
    }

    // 추종 거리를 반환하는 함수
    double getLADistance() const {
        return lookaheadDistance_;
    }

    // 차량의 바퀴 간 거리를 반환하는 함수
    float getWheelBase() const {
        return wheel_base_;
    }

    // 계산된 목표 지점을 반환하는 함수
    Point getTargetPoint() const {
        return targetPoint_;
    }
};

// Stanley 제어기 클래스 정의
// 차량의 경로 추적을 위해 스탠리 제어 방식을 사용하는 클래스
class Stanley {
private:
    double kp_; // 가로 편차 (cross track error)에 대한 이득
    double kd_; // 가속도에 대한 이득
    double target_heading_; // 목표 방향
    double prev_cross_track_error_; // 이전 가로 편차 값

    double cross_track_error_; // 현재 가로 편차
    float speed_; // 현재 차량 속도
    double curvature_; // 경로 곡률 (추가 구현 가능)
    double heading_gain_; // 방향 오차에 대한 이득
    std::vector<Point> path_; // 주어진 경로
    Point current_odom_; // 현재 차량 위치 (Odometry 데이터)

public:
    // 생성자: 변수 초기화
    Stanley()
        : kp_(1.6),
          kd_(1.5),
          target_heading_(0.0),
          prev_cross_track_error_(0.0),
          cross_track_error_(0.0),
          speed_(0.0),
          curvature_(0.0),
          heading_gain_(1.0),
          current_odom_(0,0) {
    }

    // 두 점 사이의 유클리드 거리 계산 함수
    double distance(const Point& p1, const Point& p2) {
        return std::sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
    }

    // 경로 상에서 가장 가까운 두 점을 찾는 함수
    std::pair<Point, Point> closestTwoPoints() {
        double minDistance = 1e9; // 최소 거리 추적용 임의의 큰 값
        double secondMinDistance = 1e9; // 두 번째로 가까운 점을 찾기 위한 값
        Point closestPoint;
        Point secondClosestPoint;

        for (const Point& p : path_) {
            double dist = distance(current_odom_, p); // 현재 차량 위치에서 경로 상 점까지의 거리 계산
            if (dist < minDistance) {
                secondMinDistance = minDistance;
                secondClosestPoint = closestPoint; // 두 번째로 가까운 점 업데이트

                minDistance = dist;
                closestPoint = p; // 가장 가까운 점 업데이트
            } else if (dist < secondMinDistance) {
                secondMinDistance = dist;
                secondClosestPoint = p; // 두 번째로 가까운 점 업데이트
            }
        }

        return {closestPoint, secondClosestPoint};
    }

    double calc_target_heading(){

        auto [A, B] = closestTwoPoints();
        double target_heading1 = atan2(A.y, A.x);
        double target_heading2 = atan2(B.y, B.x);

        target_heading_ = (target_heading1 + target_heading2) / 2;

        return target_heading_;

    }

    // 가로 편차 (cross track error) 계산 함수
    double calculateCrossTrackError() {
        auto [A, B] = closestTwoPoints(); // 경로 상 가장 가까운 두 점 A와 B를 얻음

        // 직선 AB로부터 현재 차량 위치까지의 거리를 계산하는 공식
        double numerator = (B.y - A.y) * current_odom_.x - (B.x - A.x) * current_odom_.y + B.x * A.y - B.y * A.x;
        double denominator = std::sqrt((B.y - A.y)*(B.y - A.y) + (B.x - A.x)*(B.x - A.x));

        if (denominator == 0.0) {
            return 0.0; // 두 점이 동일한 경우 예외 처리 (거리가 0인 경우)
        }

        cross_track_error_ = numerator / denominator; // 가로 편차 계산

        return cross_track_error_;
    }

    // Stanley 제어기에서 사용하는 방향 이득을 설정하는 함수
    void setHeadingGain(double h_gain) {
        heading_gain_ = h_gain;
    }

    // Stanley 제어기에서 사용하는 데이터를 설정하는 함수
    void setStanleyData(float speed, const Point& current_odom, const std::vector<Point>& path) {
        speed_ = speed;
        current_odom_ = current_odom;
        path_ = path;
    }

    // Stanley 제어를 통한 조향 각도 계산
    double calcStanleySteer() {
        calculateCrossTrackError(); // 가로 편차 계산

        double yaw_term = normalize_angle(target_heading_ ); // 방향 오차 계산
        double crosstrack_term = (speed_ > 0.1) ? std::atan2(kp_ * cross_track_error_, speed_) : 0.0;
        double steering = yaw_term + crosstrack_term; // 최종 조향 각도 계산

        return steering * heading_gain_; // 방향 이득을 적용하여 반환
    }
};

// Pure Pursuit과 Stanley 제어기를 결합한 제어기 클래스
class CombinedController {
private:
    PurePursuit pp_; // Pure Pursuit 제어기
    Stanley stanley_; // Stanley 제어기
    float com_steer_alpha_; // 두 제어기의 결합 비율
    double delta_max_; // 조향 각도의 최대값

public:
    // 생성자: 변수 초기화
    CombinedController()
        : com_steer_alpha_(1.0), // 기본적으로 Pure Pursuit의 비율을 0.1로 설정
          delta_max_(M_PI) { // 조향 각도의 최대값을 180도
    }

    // Pure Pursuit과 Stanley 제어기의 이득 및 설정값을 설정하는 함수
    void setPPStanleyGain(double pp_gain = 1.0, float lookahead = 1.2, double stanley_heading_gain = 1.0) {
        pp_.setPPGain(pp_gain, lookahead); // Pure Pursuit의 이득 및 추종 거리 설정
        stanley_.setHeadingGain(stanley_heading_gain); // Stanley 제어기의 방향 이득 설정
    }

    // 조향 각도의 최대값을 설정하는 함수
    void setSteerMax(double steer_max) {
        delta_max_ = steer_max;
    }

    // 결합된 제어기에서 사용할 데이터를 설정하는 함수
    void setCombinedData(float speed, const Point& current_odom, const std::vector<Point>& path) {
        pp_.setPPData(speed, current_odom, path); // Pure Pursuit 데이터 설정
        stanley_.setStanleyData(speed, current_odom, path); // Stanley 제어기 데이터 설정
    }

    // 결합된 제어기를 통한 조향 각도 계산
    double calcCombinedSteer() {
        double pp_steer = pp_.calcPPSteer(); // Pure Pursuit을 통한 조향 각도 계산
        double stanley_steer = stanley_.calcStanleySteer(); // Stanley 제어기를 통한 조향 각도 계산

        pp_steer = clip(pp_steer, -delta_max_, delta_max_); // Pure Pursuit 조향 각도를 최대값으로 제한
        stanley_steer = clip(stanley_steer, -delta_max_, delta_max_); // Stanley 제어기 조향 각도를 최대값으로 제한

        double combined_steer = com_steer_alpha_ * pp_steer + (1.0 - com_steer_alpha_) * stanley_steer; // 결합 비율에 따라 두 제어기의 조향 각도를 결합

        if(combined_steer > 0){
            combined_steer *= -3.0;
        }else if(combined_steer < 0 ){
            combined_steer *= -3.0;
        }

        return combined_steer;
    }
};

#endif // LAT_CONTROLLERS_HPP
