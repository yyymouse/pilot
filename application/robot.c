#include "bsp_init.h"
#include "robot.h"

#include "dmmotor.h"
#include "robot_def.h"
#include "robot_task.h"
#include "dmmotor.h"
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"



void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();

    RobotCMDInit();
    ShootInit();
    GimbalInit();

     DMMotorControlInit();

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
    RobotCMDTask();
    ShootTask();
    GimbalTask();

}