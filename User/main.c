#include "stm32f10x.h"
#include "debugging_USART.h"
#include "ADC_Battery.h"
#include "GPIO_LED.h"
#include "SYSTEM_TIM.h"
#include "Task_Manager.h"
#include "Motor_PWM.h"
#include "Motor_Encoder.h"
#include "I2C_MPU6050.h"
#include "ComplementaryFilter.h"
#include "EncoderFeedback.h"
#include "Control_Arrangement.h"
#include <stdio.h>

/**
 * ================================================================
 * 平衡车项目  - 主函数
 * 基于"五层塔 + 防御性编程"架构理论
 * ================================================================
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │                     五层塔架构总览                          │
 * ├─────────────────────────────────────────────────────────────┤
 * │  监控层   → 低压保护 / 角度超限急停 / 看门狗 / LED电量指示  │
 * │  调度层   → TIM3时基(1ms) / 前后台任务调度 / flag机制       │
 * │  算法层   → 串级PID（角速度环→角度环→速度环）               │
 * │  反馈处理层→ MPU6050欧拉角解算 / 编码器M/T测速 / ADC换算    │
 * │  驱动层   → GPIO/USART/TIM/ADC/I2C 硬件激活与读写           │
 * └─────────────────────────────────────────────────────────────┘
 *
 * ================================================================
 * 【五层塔理论的核心原则】
 * ================================================================
 *
 * 1. 关注点分离（Separation of Concerns）
 *    每层只做一件事，只通过接口和相邻层沟通，
 *    不需要知道隔层在做什么。
 *    → 驱动层不知道PID，算法层不知道寄存器
 *
 * 2. 需求自上而下推导，约束自下而上上报
 *    监控层定义"需要什么"，驱动层决定"能做什么"
 *    → 为什么选TIM3：不是TIM3有什么特别，
 *      而是TIM1/2/4都被占用了，TIM3是唯一空闲的
 *
 * 3. 接口优先于实现
 *    先定义每层对外暴露什么（.h文件），再填充实现（.c文件）
 *    → 数据藏在.c里（static），接口暴露在.h里（函数声明）
 *
 * 4. 面向接口编程
 *    上层调用接口函数，不直接操作下层的内部变量
 *    → main.c只调用Battery_GetFlag()，不直接读JEOC_Status
 *
 * ================================================================
 * 【防御性编程原则】
 * ================================================================
 *
 * 1. 参数入口检查
 *    函数入口验证参数合法性，非法参数立即报错返回
 *    → SendBytes: if(array==NULL || size==0) report_error(...)
 *
 * 2. 涉及中断的变量必须加volatile
 *    禁止编译器优化，强制每次从内存读取最新值
 *    → static volatile uint32_t System_Tick
 *    → static volatile uint16_t Voltage_Anal
 *
 * 3. 错误报告函数只依赖最底层
 *    防止递归调用导致栈溢出
 *    → report_error只用SendByte，不用SendString
 *
 * 4. flag消费型设计
 *    中断举旗，主循环消费后自动清零，防止重复处理
 *    → Battery_GetFlag()读完自动清零JEOC_Status
 *
 * 5. 局部变量缓存接口返回值
 *    同一数据只调用一次接口，避免重复调用和数据不一致
 *    → uint16_t raw = Battery_GetVoltageAnal()
 *      然后用raw做判断，而不是多次调用Battery_GetVoltageAnal()
 *
 * ================================================================
 * 【外设地图（来自原理图分析）】
 * ================================================================
 *
 * 功能              引脚        外设            总线
 * ─────────────────────────────────────────────────
 * 调试串口TX        PA2         USART2          APB1
 * 调试串口RX        PA3         USART2          APB1
 * 电量LED1          PA4         GPIO            APB2
 * 电量LED2          PA5         GPIO            APB2
 * 电量LED3          PA6         GPIO            APB2
 * 电池电压采集      PB0         ADC1_CH8        APB2
 * ADC触发定时器     无引脚      TIM2_TRGO       APB1
 * 系统时基          无引脚      TIM3_Update     APB1
 * 左电机PWM         PA8         TIM1_CH1        APB2  （待开发）
 * 右电机PWM         PB6         TIM4_CH1        APB1  （待开发）
 * 左编码器A/B       PB14/PB15   TIM（编码器模式）APB1  （待开发）
 * 右编码器A/B       PB3/PB4     TIM（编码器模式）APB1  （待开发）
 * MPU6050 SDA/SCL  I2C1        PB6/PB7         APB1  （待开发）
 * 蓝牙模块          USART3      -               APB1  （待开发）
 *
 * 定时器资源分配：
 *   TIM1 → 左电机PWM（占用）
 *   TIM2 → ADC触发TRGO（占用）
 *   TIM3 → 系统时基（占用）
 *   TIM4 → 右电机PWM（占用）
 *
 * ================================================================
 * 【前后台调度模型】
 * ================================================================
 *
 * 前台（中断，紧迫任务）：
 *   TIM3_IRQHandler  → 每1ms，System_Tick++
 *   ADC1_2_IRQHandler→ 每10ms，读ADC结果，举flag
 *   （未来）编码器中断 → 每Xms，读编码器计数
 *
 * 后台（主循环，非紧迫任务）：
 *   检查Battery_GetFlag() → 比较阈值，控制LED，打印电压
 *   检查tick间隔          → 按需执行各周期任务
 *
 * 中断原则：越短越好，只做采数据/举旗，业务逻辑放主循环
 *
 * 与CODESYS的类比：
 *   CODESYS Task（周期执行）↔ 主循环 + tick判断
 *   CODESYS Event Task     ↔ 中断IRQHandler
 *   CODESYS全局变量(GVL)   ↔ static volatile共享变量
 *   CODESYS FB封装         ↔ .c/.h模块封装
 *   PLC扫描周期自动管理    ↔ TIM3时基手动实现
 *
 * ================================================================
 * 【开发方法论：从需求推导到代码】
 * ================================================================
 *
 * 正确顺序：
 *   1. 读原理图 → 建立外设地图
 *   2. 确定功能需求 → 从监控层往下推
 *   3. 每层问：需要什么？约束是什么？
 *   4. 驱动层：照着外设地图配置寄存器
 *   5. 验收：用调试串口打印数据确认
 *
 * 错误顺序（照着教程抄）：
 *   直接看代码 → 不知道为什么 → 换块板子就不会了
 *
 * 例：为什么ADC用注入组？
 *   监控层需要：定时采电压
 *   调度层决定：TIM2_TRGO触发
 *   驱动层查表：TIM2_TRGO只在注入组触发源列表里
 *   结论：硬件约束决定必须用注入组，不是随意选的
 *
 * ================================================================
 * 【当前完成的模块】
 * ================================================================
 *
 *   调试串口    ✅  USART2，115200 8N1，PA2/PA3
 *   ADC电压检测 ✅  TIM2_TRGO触发，10ms采样，PB0/ADC1_CH8
 *   LED电量指示 ✅  PA4/PA5/PA6，四档显示
 *   TIM3时基    ✅  1ms心跳，System_Tick累加
 *
 * 【待开发模块】
 *
 *   编码器测速 ⬜  TIM编码器模式，M/T法测速
 *   MPU6050    ⬜  I2C读取，互补滤波解算欧拉角
 *   电机PWM    ⬜  TIM1/TIM4输出PWM，TB6612驱动
 *   串级PID    ⬜  角速度环（1ms）→ 角度环（5ms）→ 速度环（20ms）
 *   蓝牙遥控   ⬜  USART3接收指令，解析速度和转向
 * ================================================================
 */
int main(void)
{
    /* [1] NVIC优先级分组（全局配置，必须最先调用）
     * Group_2：抢占优先级2位(0~3)，子优先级2位(0~3)
     *
     * 优先级规划（抢占优先级，数字越小越高）：
     *   0 → TIM3时基（最高，1ms精度不能被打断）
     *   1 → 编码器/MPU6050（控制相关，实时性要求高）
     *   2 → ADC电压检测（慢速监控，偶尔延迟没影响）
     *   3 → 串口接收（交互任务，实时性要求最低） */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /* [2] LED GPIO初始化
     * 驱动层：PA4/PA5/PA6推挽输出
     * 服务于：监控层的电量指示逻辑 */
    LED_GPIO_Init();
    /* [3] 调试串口初始化
     * 驱动层：USART2，115200 8N1
     * 调试通道是开发的第一步，尽早初始化
     * 贯穿所有层：每层都可以通过串口输出调试数据 */
    Debugging_USART_Init();
    Debugging_USART_SendString("step1: USART OK\r\n");
    /* [4] ADC触发定时器初始化
     * 驱动层：TIM2，10ms周期，TRGO触发ADC注入组
     * 为什么TIM2_TRGO：注入组触发源列表里有TIM2_TRGO，规则组没有 */
    ADC_Battery_TIM_Init();
    Debugging_USART_SendString("step2: ADC_TIM OK\r\n");
    /* [5] ADC初始化
     * 驱动层：ADC1注入组，通道8(PB0)，右对齐，JEOC中断
     * 分压比：3.3/8.4，满电8.4V→ADC 4095 */
    ADC_Battery_Init();
    Debugging_USART_SendString("step3: ADC OK\r\n");    
    /* [6] ADC中断控制器初始化（抢占优先级2）*/
    NVIC_Battery_Init();
    Debugging_USART_SendString("step4: NVIC_Battery OK\r\n");
    /* [7] 系统时基初始化*/
    System_Init();
    Debugging_USART_SendString("step6: System OK\r\n");
    /* [9] 电机驱动初始化 */
    Double_Motors_Init();
    Debugging_USART_SendString("step7: Motors OK\r\n");
    /* [10] 编码器初始化 */
    Encoder_Init();
    Debugging_USART_SendString("step8: Encoder OK\r\n");
    /* [11] MPU6050初始化 */
    I2C_MPU6050_Init();
    Debugging_USART_SendString("step9: MPU6050 OK\r\n");
    /* [12] 互补滤波初始化 */
    ComplementaryFilter_Init();
    Debugging_USART_SendString("step10: Filter OK\r\n");
    /* [13] 控制层初始化 */
    Control_Init();
    Debugging_USART_SendString("step11: Control OK\r\n");
    /* ============================================================
     * 主循环（后台调度器）
     * 顺序扫描所有任务，各任务自己判断是否该执行
     * 不是抢占式，是协作式——谁的时间到了谁就跑
     * ============================================================ */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    while(1)
    {
        Task_Manager_Run();  // 调度层唯一入口，所有任务在此统一调度
    }
}

    /* [7] 系统时基初始化
     * 调度层核心：TIM3，1ms周期，Update中断
     * 所有任务的时间基准，相当于PLC的扫描周期引擎 */
    // TIM_System_Schedule();
    /* [8] 时基中断控制器初始化（抢占优先级0，最高）
     * 时基精度影响所有任务调度，必须最高优先级 */
    // NVIC_System_Init();




    