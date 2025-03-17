#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <std_msgs/msg/bool.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// 핀 정의
#define INTL2 2 // GO 핀
#define INTL1 3 // BACK 핀
#define ACTUATOR_PIN 4 // 액추에이터 제어 핀

#define CAR_MODE 5 // 자동차 모드 핀
bool car_signal = true;

#define HST1 6 // HST1 핀
bool h_signal_1 = true;
bool h_cnt1 = true;

#define EST 8 // EST 핀
bool EST_signal = false;

bool state_cnt_on = true;
bool state_cnt_off = false;

bool brake = false;

// micro-ROS 관련 변수
rcl_subscription_t subscriber;
std_msgs__msg__Bool msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// 브레이크 콜백 함수
void brake_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    brake = msg->data;
}

void setup() {
    set_microros_transports(); // micro-ROS 통신 설정
    Serial.begin(115200); // 시리얼 통신 시작

    pinMode(INTL1, OUTPUT); // 핀 모드 설정
    pinMode(INTL2, OUTPUT);
    pinMode(ACTUATOR_PIN, OUTPUT);

    pinMode(CAR_MODE, INPUT_PULLUP); // 입력 핀 모드 설정
    pinMode(HST1, INPUT_PULLUP);
    pinMode(EST, INPUT_PULLUP);

    Calibration(true); // 초기 보정 수행

    allocator = rcl_get_default_allocator(); // 기본 할당자 설정

    // micro-ROS 초기화
    rclc_support_init(&support, 0, NULL, &allocator);

    // 노드 생성
    rclc_node_init_default(&node, "actuator_controller", "", &support);

    // 구독자 생성
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "stop"
    );

    // 실행기 생성
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &brake_callback, ON_NEW_DATA);
}

void loop() {
    // micro-ROS 스핀
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    EST_signal = digitalRead(EST); // EST 신호 읽기
    car_signal = digitalRead(CAR_MODE); // 자동차 모드 신호 읽기
    h_signal_1 = digitalRead(HST1); // HST1 신호 읽기
    
    if (car_signal == false) {
        h_cnt1 = true;
        if (state_cnt_off == true) {
            Calibration(true); // 초기 보정 수행
            state_cnt_on = true;
            state_cnt_off = false;
        }
    } else if (car_signal == true) {
        if ((h_signal_1 == false) || (EST_signal == false) || (brake == true)) {
            if ((state_cnt_on == true) && (h_cnt1 == true)) {
                Stop(true); // 멈춤 수행
                delay(3000);
                Stop(false);
                Calibration(true); // 초기 보정 수행
                state_cnt_on = false;
                state_cnt_off = true;
                h_cnt1 = false;    
            }
        } else if ((h_signal_1 == true) && (EST_signal == true) && (h_cnt1 == true) && (brake == false)) {
            state_cnt_on = true;
            state_cnt_off = true;
        }
    }
}

// 정지 함수
void Stop(bool x) {
    if (x == true) {
        digitalWrite(INTL2, HIGH); // GO 핀 HIGH
        digitalWrite(INTL1, LOW); // BACK 핀 LOW
        analogWrite(ACTUATOR_PIN, 255); // 액추에이터 핀에 255 출력
        delay(600); // 0.6초 정지
        analogWrite(ACTUATOR_PIN, 0); // 액추에이터 핀에 0 출력
    } else if (x == false) {
        digitalWrite(INTL1, HIGH); // BACK 핀 HIGH
        digitalWrite(INTL2, LOW); // GO 핀 LOW
        analogWrite(ACTUATOR_PIN, 255); // 액추에이터 핀에 255 출력
        delay(5000); // 5초 정지
        analogWrite(ACTUATOR_PIN, 0); // 액추에이터 핀에 0 출력
    }
    return;
}

// 보정 함수
void Calibration(bool x) {
    if (x == true) {
        digitalWrite(INTL1, HIGH); // BACK 핀 HIGH
        digitalWrite(INTL2, LOW); // GO 핀 LOW
        analogWrite(ACTUATOR_PIN, 255); // 액추에이터 핀에 255 출력
        delay(5000); // 5초 보정
        analogWrite(ACTUATOR_PIN, 0); // 액추에이터 핀에 0 출력
        digitalWrite(INTL2, HIGH); // GO 핀 HIGH
        digitalWrite(INTL1, LOW); // BACK 핀 LOW
        analogWrite(ACTUATOR_PIN, 255); // 액추에이터 핀에 255 출력
        delay(2400); // 2.4초 보정
        analogWrite(ACTUATOR_PIN, 0); // 액추에이터 핀에 0 출력
    }
    return;
}
