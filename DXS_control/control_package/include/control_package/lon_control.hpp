#ifndef LON_CONTROLLERS_HPP
#define LON_CONTROLLERS_HPP

#include <iostream>
#include <memory>
#include <algorithm>
#include <cmath>
#include "PID_control.hpp"


class LonController {
private:
    float target_speed_;      // 목표 속도
    double curvature_;        // 도로의 곡률

public:
    // 생성자: 멤버 변수 초기화
    LonController()
        : target_speed_(100.0f),
          curvature_(0.0) {
    }

    // 곡률 값 설정
    void setCurvature(double curvature) {
        curvature_ = curvature;
    }

    // 입력된 목표 속도와 현재 상태에 따라 최종 목표 속도 계산
    int targetSpeed(int input_target_speed = 100) {
        target_speed_ = input_target_speed;

        if (curvature_ > 0.0 && curvature_ <= 0.1) {
            target_speed_ *= 0.8f;
        } else if (curvature_ <= 0.2) {
            target_speed_ *= 0.6f;
        } else if (curvature_ <= 0.3) {
            target_speed_ *= 0.5f;
        }

        // 목표 속도를 0에서 255 사이로 제한
        target_speed_ = std::clamp(target_speed_, 0.0f, 255.0f);

        return target_speed_;
    }
};

#endif // LON_CONTROLLERS_HPP
