#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cmath>
#include <algorithm>


class Point {
public:
    double x;
    double y;

    Point() : x(0.0), y(0.0) {}
    Point(double x_val, double y_val) : x(x_val), y(y_val) {}
};

// 저역 통과 필터 함수
inline double low_pass_filter(double val, double pre_val, double alpha) {
    return val * alpha + pre_val * (1.0 - alpha);
}

// 각도 정규화 함수
inline double normalize_angle(double rad_angle) {
    while (rad_angle > M_PI)
        rad_angle -= 2.0 * M_PI;
    while (rad_angle < -M_PI)
        rad_angle += 2.0 * M_PI;
    return rad_angle;
}

// 템플릿 함수로 값 제한
template <typename T>
inline T clip(const T& value, const T& min_val, const T& max_val) {
    return std::max(min_val, std::min(value, max_val));
}

class PIDController {
public:
    PIDController(double min_output, double max_output)
        : kp_(0.0), ki_(0.0), kd_(0.0),
          min_output_(min_output), max_output_(max_output),
          integral_(0.0), prev_error_(0.0),
          pre_derivative_(0.0) {
    }

    void setPIDGain(double kp, double kd) {
        kp_ = kp;
        ki_ = 0.0; // 편의상 0으로 설정
        kd_ = kd;
    }

    double compute(double target_value, double measured_value, double dt = 0.05, double alpha = 0.1) {
        double error = target_value - measured_value; // 오차 계산
        integral_ += error * dt; // 적분항 계산

        // 미분항 계산 및 저역 통과 필터 적용
        double derivative = (error - prev_error_) / dt;
        derivative = low_pass_filter(derivative, pre_derivative_, alpha);
        pre_derivative_ = derivative;

        prev_error_ = error;

        // PID 출력 계산
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 출력값 제한
        output = clip(output, min_output_, max_output_);

        return output;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double min_output_;
    double max_output_;
    double integral_;
    double prev_error_;
    double pre_derivative_;
};

#endif // PID_CONTROLLER_HPP
