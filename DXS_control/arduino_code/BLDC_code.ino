#include <micro_ros_arduino.h>
#include <std_msgs/msg/u_int8.h>  // UInt8 메시지 타입
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

#define BLDC_T 2
volatile long val = 0;
volatile long th_val = 0;
volatile long BLDC_Speed = 0; // long 타입으로 오버플로우 방지

#define CAR_MODE 5 // 자동 모드
bool car_signal = true;

#define HST1 6 // 센서 1 핀 정의
bool h_signal_1 = true; // 센서 1 신호 변수
bool h_cnt1 = true; // 센서 1 카운트 변수

#define EST 8
bool EST_signal = true;

bool state_cnt_on = true;
bool state_cnt_off = true;

bool brake = false; // brake 상태 변수
bool light_brake = false; // light_brake 상태 변수

rcl_subscription_t subscriber_speed;   // 속도 구독자
rcl_subscription_t subscriber_brake;   // 브레이크 구독자
rcl_subscription_t subscriber_light_brake;  // light_brake 구독자

std_msgs__msg__UInt8 incoming_msg;     // 속도 메시지
std_msgs__msg__Bool brake_msg;         // 브레이크 메시지
std_msgs__msg__Bool light_brake_msg;   // light_brake 메시지

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// 속도 메시지 콜백 함수
void subscription_callback(const void * msgin) {
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    BLDC_Speed = (long)msg->data; // uint8 값을 long으로 변환하여 저장
}

// 브레이크 메시지 콜백 함수
void brake_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    brake = msg->data;
}

// light_brake 메시지 콜백 함수
void light_brake_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    light_brake = msg->data;
}

void setup() {
    Serial.begin(115200);
    set_microros_transports();

    // Micro-ROS 초기화
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "lon_node", "", &support); // 노드 이름 정의

    // 속도 구독자 초기화
    rclc_subscription_init_default(
        &subscriber_speed,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),  // UInt8 메시지 타입
        "linear_cmd" // 주제 이름
    );

    // 브레이크 구독자 초기화
    rclc_subscription_init_default(
        &subscriber_brake,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),  // Bool 메시지 타입
        "stop" // 브레이크 제어 주제 이름
    );

    // light_brake 구독자 초기화
    rclc_subscription_init_default(
        &subscriber_light_brake,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),  // Bool 메시지 타입
        "start" // light_brake 제어 주제 이름
    );

    // Executor 초기화 (3개의 구독자 추가)
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(
        &executor, 
        &subscriber_speed, 
        &incoming_msg, 
        &subscription_callback, 
        ON_NEW_DATA
    );
    rclc_executor_add_subscription(
        &executor, 
        &subscriber_brake, 
        &brake_msg, 
        &brake_callback, 
        ON_NEW_DATA
    );
    rclc_executor_add_subscription(
        &executor, 
        &subscriber_light_brake, 
        &light_brake_msg, 
        &light_brake_callback, 
        ON_NEW_DATA
    );

    // 핀 설정
    pinMode(BLDC_T, OUTPUT);
    pinMode(CAR_MODE, INPUT_PULLUP);
    pinMode(HST1, INPUT_PULLUP);
    pinMode(EST, INPUT_PULLUP);
}

void loop() {
    // 핀에서 신호를 읽기
    car_signal = digitalRead(CAR_MODE);
    h_signal_1 = digitalRead(HST1);
    EST_signal = digitalRead(EST);

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    if (car_signal == false) { // 정지 상태
        Stop(true);
        h_cnt1 = true;
    } else if (car_signal == true) { // 주행 상태
        if (light_brake == true) {  // 경고등 브레이크
            Stop(true);
            h_cnt1 = true;
        } else if (light_brake == false) {
            if ((h_signal_1 == false) || (EST_signal == false) || (brake == true)) {
                if (h_cnt1 == true) {
                    h_cnt1 = false;
                    Stop(true);
                }
            } else if ((h_signal_1 == true) && (EST_signal == true) && (h_cnt1 == true)) {
                Go(BLDC_Speed);
                state_cnt_on = true;
                state_cnt_off = true;
            }
        }
    }
}

// 속도 제어 함수
void Go(long x) {
    x = constrain(x, 0, 255);  // PWM 신호 범위 (0~255)
    analogWrite(BLDC_T, x);
}

// 정지 제어 함수
void Stop(bool x) {
    analogWrite(BLDC_T, 0);  // BLDC 모터 정지 (0으로 설정)
}
