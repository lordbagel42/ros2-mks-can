#pragma once

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/empty.h>

#include "config.h"
#include "motor_manager.h"

// 1 timer + 4 subscribers
#define EXECUTOR_HANDLES  5

enum MicroROSState {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

/// Manages the micro-ROS node, publishers, subscribers, and timer.
class UROSInterface {
public:
    explicit UROSInterface(MotorManager& motors);

    /// Set up the serial transport.
    void begin();

    /// State-machine tick — call once per loop().
    void spin();

    MicroROSState getState() const { return _state; }

private:
    MotorManager&  _motors;
    MicroROSState  _state = WAITING_AGENT;

    // ── RCL entities ─────────────────────────────────────────────────────
    rcl_allocator_t _allocator;
    rclc_support_t  _support;
    rcl_node_t      _node;
    rclc_executor_t _executor;

    rcl_publisher_t    _joint_state_pub;
    rcl_subscription_t _pos_cmd_sub;
    rcl_subscription_t _vel_cmd_sub;
    rcl_subscription_t _enable_sub;
    rcl_subscription_t _estop_sub;
    rcl_timer_t        _telemetry_timer;

    // ── Messages (static allocation) ─────────────────────────────────────
    sensor_msgs__msg__JointState         _joint_state_msg;
    std_msgs__msg__Float64MultiArray     _pos_cmd_msg;
    std_msgs__msg__Float64MultiArray     _vel_cmd_msg;
    std_msgs__msg__Bool                  _enable_msg;
    std_msgs__msg__Empty                 _estop_msg;

    // Backing storage for JointState arrays
    double                      _js_pos[MAX_MOTORS];
    double                      _js_vel[MAX_MOTORS];
    double                      _js_eff[MAX_MOTORS];
    rosidl_runtime_c__String    _js_names[MAX_MOTORS];
    char                        _js_name_buf[MAX_MOTORS][16];
    char                        _frame_id_buf[16];

    // Backing storage for command arrays
    double _pos_cmd_data[MAX_MOTORS];
    double _vel_cmd_data[MAX_MOTORS];

    // ── Helpers ──────────────────────────────────────────────────────────
    bool createEntities();
    void destroyEntities();
    void initMessages();
    void publishJointState();

    // ── Static callbacks ─────────────────────────────────────────────────
    static UROSInterface* _instance;
    static void telemetryCb(rcl_timer_t* timer, int64_t last_call_time);
    static void positionCmdCb(const void* msg);
    static void velocityCmdCb(const void* msg);
    static void enableCmdCb(const void* msg);
    static void estopCmdCb(const void* msg);
};
