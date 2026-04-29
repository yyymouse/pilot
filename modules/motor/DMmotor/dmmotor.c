#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop_type, Feedback_Source_e feed_source)
{
    switch (loop_type)
    {
    case ANGLE_LOOP:
        motor->motor_settings.angle_feedback_source = feed_source;
        break;
    case SPEED_LOOP:
        motor->motor_settings.speed_feedback_source = feed_source;
        break;
    }
}


static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->raw_angle = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16) * RAD_2_DEGREE;

    float delta = measure->raw_angle - measure->last_raw_angle;
    while (delta > 180.0f)  delta -= 360.0f * 2.0f;
    while (delta < -180.0f) delta += 360.0f * 2.0f;

    measure->total_angle += delta;
    measure->last_raw_angle = measure->raw_angle;

    float pos = fmodf(measure->total_angle, 360.0f);
    if (pos < 0.0f) pos += 360.0f;
    measure->position = pos;

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12) * RAD_2_DEGREE;

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
    // if (measure->last_position - measure->position > 180.0f)
    // {
    //     measure->total_round++;
    // }else if (measure->last_position - measure->position < -180.0f)
    // {
    //     measure->total_round--;
    // }
    // measure->total_angle = measure->total_round * 360.0f + measure->position;
}

static void DMMotorLostCallback(void *motor_ptr)
{
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    dm_motor_instance[idx++] = motor;

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    // DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);

    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void DMMotorTask(void const *argument)
{
    float pid_ref, pid_measure;
    DMMotorInstance *motor = (DMMotorInstance *)argument;   // 当前电机实例
    Motor_Control_Setting_s *setting;                       // 电机控制参数
    Motor_Controller_s *motor_controller;                   // 电机控制器
    DM_Motor_Measure_s *measure;                            // 电机测量值
    DMMotor_Send_s motor_send_mailbox;
    while (1)
    {
        setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;  //设置反转

        //位置环PID
        //计算位置环，只有开启位置环且外环为位置环的时候才会计算
        if ((setting->close_loop_type & ANGLE_LOOP) && (setting->outer_loop_type == ANGLE_LOOP))
        {
            if (setting->angle_feedback_source == OTHER_FEED)
            {
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            }else
            {
                pid_measure = measure->total_angle;
            }
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        //速度PID
        //外环为速度或者位置环且启用了速度环
        if ((setting->close_loop_type & SPEED_LOOP) && (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)  //速度环前馈
            {
                pid_ref += *motor_controller->speed_feedforward_ptr;
            }
            if (setting->angle_feedback_source == OTHER_FEED)
            {
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            }else
            {
                pid_measure = measure->velocity;
            }
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        if (setting->feedforward_flag & CURRENT_FEEDFORWARD)    //力矩前馈
        {
            pid_ref += *motor_controller->current_feedforward_ptr;
        }
        //力矩环
        if (setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->torque, pid_ref);
        }

        pid_ref *= 0.0007;

        if (motor->stop_flag == MOTOR_STOP)
        {
            pid_ref = 0;
        }

        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
        motor_send_mailbox.torque_des = float_to_uint(pid_ref, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.Kp = float_to_uint(0, DM_KP_MIN, DM_KP_MAX, 12);
        motor_send_mailbox.Kd = float_to_uint(0, DM_KD_MIN, DM_KD_MAX, 12);

        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        CANTransmit(motor->motor_can_instace, 1);
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);


        osDelay(2);
    }
}
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}