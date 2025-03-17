#include <micro_ros_arduino.h>
#include <std_msgs/msg/float64.h> // Float32 메시지 타입 임포트
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

// 모터 제어 핀
const int motorDirPin1 = 9;  // W224 IN1
const int motorDirPin2 = 10; // W224 IN2
const int motorPWMPin = 11;  // W224 ENA

// 엔코더 핀
const int encoderPinA = 12;
const int encoderPinB = 13;

volatile long encoderPos = 0; // 엔코더 위치를 저장하는 변수
long errorPos = 0;            // 목표 엔코더 위치

// 엔코더 회전수와 각도 변환 비율
const float ratio = 360.0 / 81.0 / 76.0;

// P 제어 상수
float Kp = 10;     // P 제어 상수
float errorDeg = 0; // 목표 각도

#define CAR_MODE 5 // 자동 모드 핀 정의
bool car_signal = true; // 자동차 신호 변수

#define HST1 6 // 센서 1 핀 정의
bool h_signal_1 = true; // 센서 1 신호 변수
bool h_cnt1 = true;

#define EST 8 // 긴급 정지 센서 핀 정의
bool EST_signal = true; // 긴급 정지 센서 신호 변수

bool state_cnt_on = true; // 상태 카운트 ON 변수
bool state_cnt_off = true; // 상태 카운트 OFF 변수

bool brake = false; // 브레이크 상태 변수

// Micro-ROS 변수들
rcl_subscription_t subscriber;
rcl_subscription_t brake_subscriber;  // 브레이크 상태 구독자 추가
std_msgs__msg__Float64 incoming_msg;  // Float32 메시지 타입으로 변경
std_msgs__msg__Bool brake_msg;        // 브레이크 메시지 타입 추가

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// 콜백 함수
void subscription_callback(const void *msgin) {
    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin; // Float32 메시지 타입으로 변경
    errorDeg = msg->data;
    errorDeg = constrain(errorDeg, -180, 180); // 목표 각도를 ±180도로 제한
    errorPos = errorDeg / ratio;
}

// 브레이크 상태 콜백 함수
void brake_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    brake = msg->data;
}

void setup() {
    Serial.begin(115200); // 시리얼 통신 속도 조정 가능
    set_microros_transports();

    // Micro-ROS 초기화
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "lat_node", "", &support); // 노드 이름 변경

    // 각도 명령을 받는 구독자 초기화
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), // Float32 메시지 타입으로 변경
        "angular_cmd" // 각도 명령 주제
    );

    // 브레이크 상태를 받는 구독자 초기화
    rclc_subscription_init_default(
        &brake_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), // Bool 메시지 타입
        "stop" // 브레이크 명령 주제 추가
    );

    // Executor 생성 (구독자 2개 추가)
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(
        &executor,
        &subscriber,
        &incoming_msg,
        &subscription_callback,
        ON_NEW_DATA
    );
    rclc_executor_add_subscription(
        &executor,
        &brake_subscriber,
        &brake_msg,
        &brake_callback,
        ON_NEW_DATA
    );

    // 엔코더 핀 설정 및 인터럽트 초기화
    pinMode(encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
    pinMode(encoderPinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);

    // 모터 핀 설정
    pinMode(motorDirPin1, OUTPUT);
    pinMode(motorDirPin2, OUTPUT);
    pinMode(motorPWMPin, OUTPUT);

    // 기타 핀 설정
    pinMode(CAR_MODE, INPUT_PULLUP);
    pinMode(HST1, INPUT_PULLUP);
    pinMode(EST, INPUT_PULLUP);

    // 목표 위치 초기화
    errorDeg = constrain(errorDeg, -180, 180);
    errorPos = errorDeg / ratio;
}

void loop() {
    // 핀에서 신호를 읽기
    car_signal = digitalRead(CAR_MODE);
    h_signal_1 = digitalRead(HST1);
    EST_signal = digitalRead(EST);

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    long currentPos = encoderPos;
    float motorDeg = float(currentPos) * ratio;

    if (car_signal == false) {
        Stop();
        h_cnt1 = true;
    } else if (car_signal == true) {
        long currentPos = encoderPos;
        float motorDeg = float(currentPos) * ratio;
        if ((h_signal_1 == false) || (EST_signal == false) || (brake == true)) {
        if (h_cnt1 == true) {
          h_cnt1 = false;
          Stop();
          }
        } else if ((h_signal_1 == true) && (EST_signal == true) && (h_cnt1 == true) && (brake == false)) {
            float error = errorDeg - 2 * motorDeg;
            float control = Kp * error;
            bool direction = control >= 0;
            doMotor(direction, min(abs(control), 255));
        }
    }
}

void Stop() {
    analogWrite(motorPWMPin, 0);
}

void doEncoderA() {
    encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : -1;
}

void doEncoderB() {
    encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? -1 : 1;
}

void doMotor(bool dir, int vel) {
    digitalWrite(motorDirPin1, dir ? HIGH : LOW);
    digitalWrite(motorDirPin2, dir ? LOW : HIGH);
    analogWrite(motorPWMPin, vel);
}
