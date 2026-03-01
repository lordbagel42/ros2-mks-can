#include "uros_interface.h"
#include <cstring>

// Abort-free error check: returns false on failure.
#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) return false; }

UROSInterface* UROSInterface::_instance = nullptr;

// ─── Construction / setup ───────────────────────────────────────────────────

UROSInterface::UROSInterface(MotorManager& motors) : _motors(motors) {
    _instance = this;
}

void UROSInterface::begin() {
#ifdef TRANSPORT_WIFI
    set_microros_wifi_transports(
        (char*)WIFI_SSID, (char*)WIFI_PASS,
        (char*)AGENT_IP, AGENT_PORT);
#else
    set_microros_serial_transports(Serial);
#endif
    _state = WAITING_AGENT;
}

// ─── State machine ──────────────────────────────────────────────────────────

void UROSInterface::spin() {
    switch (_state) {
        case WAITING_AGENT:
            if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT, AGENT_PING_ATTEMPTS)
                    == RMW_RET_OK) {
                _state = AGENT_AVAILABLE;
            }
            break;

        case AGENT_AVAILABLE:
            _state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
            break;

        case AGENT_CONNECTED:
            if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT, AGENT_PING_ATTEMPTS)
                    != RMW_RET_OK) {
                _state = AGENT_DISCONNECTED;
                break;
            }
            _motors.processResponses();
            rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(1));
            break;

        case AGENT_DISCONNECTED:
            destroyEntities();
            _state = WAITING_AGENT;
            break;
    }
}

// ─── Entity lifecycle ───────────────────────────────────────────────────────

bool UROSInterface::createEntities() {
    _allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&_support, 0, NULL, &_allocator));
    RCCHECK(rclc_node_init_default(&_node, "mks_servo_node", "mks_servo",
                                   &_support));

    // Synchronise clock with the agent so that JointState timestamps are
    // consistent with the rest of the ROS graph.
    rmw_uros_sync_session(1000);

#if USE_BEST_EFFORT_QOS
    const rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
#else
    const rmw_qos_profile_t qos = rmw_qos_profile_default;
#endif

    // Publisher ──────────────────────────────────────────────────────────
    RCCHECK(rclc_publisher_init(
        &_joint_state_pub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states", &qos));

    // Subscribers ────────────────────────────────────────────────────────
    RCCHECK(rclc_subscription_init(
        &_pos_cmd_sub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "mks_servo/position_cmd", &qos));

    RCCHECK(rclc_subscription_init(
        &_vel_cmd_sub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "mks_servo/velocity_cmd", &qos));

    RCCHECK(rclc_subscription_init(
        &_enable_sub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "mks_servo/enable", &qos));

    RCCHECK(rclc_subscription_init(
        &_estop_sub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "mks_servo/emergency_stop", &qos));

    // Timer ──────────────────────────────────────────────────────────────
    RCCHECK(rclc_timer_init_default(
        &_telemetry_timer, &_support,
        RCL_MS_TO_NS(TELEMETRY_PERIOD_MS),
        telemetryCb));

    // Executor ───────────────────────────────────────────────────────────
    RCCHECK(rclc_executor_init(&_executor, &_support.context,
                               EXECUTOR_HANDLES, &_allocator));
    RCCHECK(rclc_executor_add_timer(&_executor, &_telemetry_timer));
    RCCHECK(rclc_executor_add_subscription(
        &_executor, &_pos_cmd_sub, &_pos_cmd_msg,
        positionCmdCb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &_executor, &_vel_cmd_sub, &_vel_cmd_msg,
        velocityCmdCb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &_executor, &_enable_sub, &_enable_msg,
        enableCmdCb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &_executor, &_estop_sub, &_estop_msg,
        estopCmdCb, ON_NEW_DATA));

    initMessages();
    return true;
}

void UROSInterface::destroyEntities() {
    rmw_context_t* ctx = rcl_context_get_rmw_context(&_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);

    rcl_publisher_fini(&_joint_state_pub, &_node);
    rcl_subscription_fini(&_pos_cmd_sub, &_node);
    rcl_subscription_fini(&_vel_cmd_sub, &_node);
    rcl_subscription_fini(&_enable_sub, &_node);
    rcl_subscription_fini(&_estop_sub, &_node);
    rcl_timer_fini(&_telemetry_timer);
    rclc_executor_fini(&_executor);
    rcl_node_fini(&_node);
    rclc_support_fini(&_support);
}

// ─── Message initialisation (zero-copy, static buffers) ─────────────────────

void UROSInterface::initMessages() {
    uint8_t n = _motors.getMotorCount();

    // JointState ─────────────────────────────────────────────────────────
    memset(&_joint_state_msg, 0, sizeof(_joint_state_msg));

    // frame_id
    strncpy(_frame_id_buf, "base_link", sizeof(_frame_id_buf));
    _joint_state_msg.header.frame_id.data     = _frame_id_buf;
    _joint_state_msg.header.frame_id.size      = strlen(_frame_id_buf);
    _joint_state_msg.header.frame_id.capacity  = sizeof(_frame_id_buf);

    // name[]
    _joint_state_msg.name.data     = _js_names;
    _joint_state_msg.name.size     = n;
    _joint_state_msg.name.capacity = MAX_MOTORS;
    for (uint8_t i = 0; i < n; i++) {
        snprintf(_js_name_buf[i], sizeof(_js_name_buf[i]), "motor_%u", i);
        _js_names[i].data     = _js_name_buf[i];
        _js_names[i].size     = strlen(_js_name_buf[i]);
        _js_names[i].capacity = sizeof(_js_name_buf[i]);
    }

    // position[], velocity[], effort[]
    _joint_state_msg.position.data     = _js_pos;
    _joint_state_msg.position.size     = n;
    _joint_state_msg.position.capacity = MAX_MOTORS;

    _joint_state_msg.velocity.data     = _js_vel;
    _joint_state_msg.velocity.size     = n;
    _joint_state_msg.velocity.capacity = MAX_MOTORS;

    _joint_state_msg.effort.data       = _js_eff;
    _joint_state_msg.effort.size       = n;
    _joint_state_msg.effort.capacity   = MAX_MOTORS;

    // Command messages ───────────────────────────────────────────────────
    memset(&_pos_cmd_msg, 0, sizeof(_pos_cmd_msg));
    _pos_cmd_msg.data.data     = _pos_cmd_data;
    _pos_cmd_msg.data.size     = 0;
    _pos_cmd_msg.data.capacity = MAX_MOTORS;

    memset(&_vel_cmd_msg, 0, sizeof(_vel_cmd_msg));
    _vel_cmd_msg.data.data     = _vel_cmd_data;
    _vel_cmd_msg.data.size     = 0;
    _vel_cmd_msg.data.capacity = MAX_MOTORS;
}

// ─── Publishing ─────────────────────────────────────────────────────────────

void UROSInterface::publishJointState() {
    uint8_t n = _motors.getMotorCount();

    for (uint8_t i = 0; i < n; i++) {
        const MotorState& s = _motors.getState(i);
        _js_pos[i] = s.position_rad;
        _js_vel[i] = s.velocity_rad_s;
        _js_eff[i] = s.effort;
    }

    // Timestamp — uses agent-synchronised epoch
    int64_t now_ms = rmw_uros_epoch_millis();
    _joint_state_msg.header.stamp.sec     = (int32_t)(now_ms / 1000);
    _joint_state_msg.header.stamp.nanosec =
        (uint32_t)((now_ms % 1000) * 1000000);

    rcl_publish(&_joint_state_pub, &_joint_state_msg, NULL);
}

// ─── Callbacks ──────────────────────────────────────────────────────────────

void UROSInterface::telemetryCb(rcl_timer_t* timer,
                                int64_t /*last_call_time*/) {
    if (!timer || !_instance) return;
    _instance->_motors.requestTelemetry();
    _instance->_motors.processResponses();
    _instance->publishJointState();
}

void UROSInterface::positionCmdCb(const void* msg) {
    if (!_instance) return;
    auto* m = (const std_msgs__msg__Float64MultiArray*)msg;
    uint8_t n = min((uint8_t)m->data.size,
                    _instance->_motors.getMotorCount());
    for (uint8_t i = 0; i < n; i++)
        _instance->_motors.cmdPosition(i, m->data.data[i]);
}

void UROSInterface::velocityCmdCb(const void* msg) {
    if (!_instance) return;
    auto* m = (const std_msgs__msg__Float64MultiArray*)msg;
    uint8_t n = min((uint8_t)m->data.size,
                    _instance->_motors.getMotorCount());
    for (uint8_t i = 0; i < n; i++) {
        if (m->data.data[i] == 0.0)
            _instance->_motors.cmdVelocityStop(i);
        else
            _instance->_motors.cmdVelocity(i, m->data.data[i]);
    }
}

void UROSInterface::enableCmdCb(const void* msg) {
    if (!_instance) return;
    auto* m = (const std_msgs__msg__Bool*)msg;
    _instance->_motors.cmdEnableAll(m->data);
}

void UROSInterface::estopCmdCb(const void* /*msg*/) {
    if (_instance) _instance->_motors.cmdEmergencyStop();
}
