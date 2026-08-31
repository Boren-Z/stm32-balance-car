#include "stm32f10x.h"
#include "MPU6050_Reg.h"
#include "I2C_MPU6050.h"

/**
 * ================================================================
 * 【五层塔定位】驱动层 - I2C/MPU6050姿态传感器驱动模块
 * ================================================================
 *
 * 本模块在五层塔中的位置：
 *
 *   反馈处理层 → 调用MPU6050_GetData()获取六轴原始数据，
 *                互补滤波算法在这一层把原始数据转换成俯仰角
 *   驱动层    ← 本模块，激活I2C1硬件外设 + MPU6050寄存器配置
 *
 * 硬件架构：
 *   STM32 I2C1(重映射) ←→ MPU6050(姿态传感器，I2C从机)
 *
 *   I2C1_SCL → PB8（重映射后，默认PB6被右电机占用）
 *   I2C1_SDA → PB9（重映射后，默认PB7被右电机占用）
 *
 *   MPU6050硬件状态：
 *     AD0   → GND  → I2C从机地址 = 0x68（7位）→ 0xD0写/0xD1读（8位总线字节）
 *     nCS   → 3V3  → 强制进入I2C模式（拉低会变成SPI模式，I2C完全失效）
 *
 * 对外接口（.h文件声明，外部只能看到这三个）：
 *   I2C_MPU6050_Init()   → Action接口，初始化I2C总线+MPU6050寄存器
 *   MPU6050_GetID()      → Get接口，验证通信是否正常（读WHO_AM_I）
 *   MPU6050_GetData()    → Get接口，读取六轴原始数据
 *
 * 内部函数（static，外部不可见）：
 *   I2C_GPIO_Init()       → I2C1引脚初始化（协议无关，换任何I2C设备都能复用）
 *   I2C1_Protocol_Init()  → I2C1总线协议参数（同样协议无关）
 *   MPU6050_WaitEvent()   → 带超时保护的I2C事件等待（防御性编程）
 *   MPU6050_WriteReg()    → 写MPU6050单个寄存器
 *   MPU6050_ReadReg()     → 读MPU6050单个寄存器（含Restart换向）
 *   MPU6050_Init()        → MPU6050专属寄存器配置（设备相关）
 *   MPU6050_Data_Merge()  → 高低字节拼接成16位有符号数
 *
 * 【五层塔封装原则】
 *   协议层(I2C_GPIO_Init/I2C1_Protocol_Init) 与 设备层(MPU6050_Init) 严格分离
 *   前者只关心"STM32怎么把I2C跑起来"，换成任何I2C设备都能直接复用
 *   后者只关心"MPU6050这个具体芯片需要被写入什么值"
 *   这和Double_Motors_Init()里GPIO初始化与PWM参数初始化分离是同一个原则
 *
 * 【两层"配置"的区别，曾经混淆过】
 *   PWR_MGMT_1  ≈ 整栋房子的总电闸（SLEEP=芯片整体睡眠/工作）
 *   PWR_MGMT_2  ≈ 每个房间各自的灯开关（六个轴各自待机/工作）
 *   总闸必须先开，房间开关才有意义
 * ================================================================
 */

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址（7位0x68左移1位，最低位留给R/W）

/**
 * ================================================================
 * [驱动层-内部] I2C1引脚初始化（static，与MPU6050无关，协议层）
 * ================================================================
 *
 * 职责：把I2C1重映射到PB8/PB9，配置为复用开漏模式
 *
 * 【为什么必须是开漏（AF_OD），不能是推挽】
 *   I2C总线靠外部上拉电阻+开漏输出实现"线与"逻辑：
 *   只要有一个设备拉低，总线就是低电平；都不拉低，靠上拉电阻回高电平
 *   这样多个设备才能共享同一条SDA/SCL，互不冲突
 *   推挽输出做不到"线与"，会导致总线冲突
 *
 * 【曾犯的错误】
 *   GPIO_PinRemapConfig()之前忘记开启RCC_APB2Periph_AFIO时钟
 *   不开AFIO时钟，重映射配置不会生效，I2C1依然停留在默认PB6/PB7
 *   （这和Encoder模块解除JTAG占用PB3/PB4时是同一个坑，重映射前必须先开AFIO时钟）
 * ================================================================
 */
static void I2C_GPIO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO  , ENABLE);   // 重映射前必须先开AFIO时钟
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);             // 默认PB6/PB7被右电机占用，必须重映射

    GPIO_InitTypeDef    GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;            // 复用开漏，I2C硬件特性要求
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * ================================================================
 * [驱动层-内部] I2C1总线协议参数初始化（static，与MPU6050无关，协议层）
 * ================================================================
 *
 * 职责：配置I2C1的通信速率、模式、应答规则
 * 注意：这里的OwnAddress1是STM32自己的地址，只在STM32被当作"从机"时才用得到
 *       本项目STM32永远是主机，这个值填什么都不影响实际通信
 *
 * 【命名的演变】
 *   最初命名为I2C_STM_MPU6050_Init，名字暗示和MPU6050有关
 *   实际内部完全不涉及MPU6050的任何寄存器，纯粹是STM32侧的协议配置
 *   改名为I2C1_Protocol_Init，让名字和职责真正对应
 *
 * 【曾犯的错误】
 *   1. 函数名直接叫I2C_Init，和标准库自带的I2C_Init()函数重名，编译冲突
 *
 *   2. I2C_OwnAddress1一度留作占位符(?)没有填具体值
 *      必须填一个合法数字（这里填0x00），不能留空
 *
 *   3. I2C_ClockSpeed最初填了20000(20kHz)
 *      原因是把手册注释里的"400kHz上限"看成了"40kHz"
 *      最终改为标准模式100000(100kHz)，更稳定可靠
 *
 *   4. I2C_DutyCycle最初配的是I2C_DutyCycle_16_9
 *      这个占空比参数只在快速模式(>100kHz)下才需要真正生效
 *      标准模式(100kHz)应该用I2C_DutyCycle_2
 *
 * 【曾经考虑过的方案：I2C_StructInit()】
 *   标准库提供I2C_StructInit()可以先填默认值、再覆盖部分字段
 *   语法上完全合法，但最终决定不用——
 *   项目里GPIO/TIM/USART/ADC/EXTI/NVIC全部是逐字段显式赋值，
 *   为了保持全项目风格一致、每个参数取值意图透明，
 *   坚持显式写法，不用StructInit简写
 * ================================================================
 */
static void I2C1_Protocol_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_ClockSpeed            = 400000;                    // 标准模式400kHz，调试阶段优先稳定性
    I2C_InitStructure.I2C_Mode                  = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle             = I2C_DutyCycle_2;           // 标准模式占空比，快速模式才需要16_9
    I2C_InitStructure.I2C_OwnAddress1           = 0x00;                      // STM32作为主机，这个值不会被实际用到
    I2C_InitStructure.I2C_Ack                   = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress   = I2C_AcknowledgedAddress_7bit;

    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}

/**
 * ================================================================
 * [驱动层-内部] I2C事件等待（static，带超时保护）
 * ================================================================
 *
 * 职责：封装"等flag再操作"这个I2C通信的核心模式
 * 每发一步（START/地址/数据），都要等对应事件确认完成，才能进行下一步
 * 类比：和USART的while(USART_GetFlagStatus(...)==RESET)是同一种模式，
 *       I2C只是把这个模式重复了更多次（协议更复杂）
 *
 * 【防御性编程】
 *   不带超时的死等write(while(...!=SUCCESS));一旦I2C总线异常（接线松动、
 *   设备无响应），程序会永久卡死在这一行
 *   加入Timeout计数，最坏情况下也能跳出循环，不会让整个系统失去响应
 *   （这个写法参考了网上找到的MPU6050驱动范例，原理与USART的
 *    Receivebyte()缺少超时保护是同一类问题，这里做了改进）
 *
 * 【设计简化】
 *   参考代码里这个函数带I2C_TypeDef*参数，可以兼容I2C1/I2C2
 *   本项目固定只用I2C1，所以直接去掉了这个参数，简化接口
 * ================================================================
 */
static void MPU6050_WaitEvent(uint32_t I2C_EVENT)
{
    uint32_t Timeout = 10000;
    while(I2C_CheckEvent(I2C1, I2C_EVENT) != SUCCESS)
    {
        Timeout--;
        if(Timeout == 0) break;   // 超时强制跳出，不死等
    }
}

/**
 * ================================================================
 * [驱动层-内部] MPU6050写寄存器（static）
 * ================================================================
 *
 * I2C写时序：START → 地址(写) → EV6 → reg字节 → EV8 → data字节 → EV8_2 → STOP
 *
 * 【曾犯的错误】
 *   1. 第一版完全遗漏了reg这个字节，只发送了data
 *      没有意识到"写寄存器"本质是发两个字节：
 *      第一个字节告诉MPU6050"操作哪个寄存器"，第二个才是真正写入的值
 *      （这个"寄存器地址"概念本身不属于I2C协议，是MPU6050自己的私有约定，
 *       I2C协议本身只认"数据字节"，不知道"寄存器"是什么）
 *
 *   2. I2C_GenerateSTOP()一度被放在等待发送完成之前
 *      数据可能还没真正发出去，总线就被提前掐断
 *      正确顺序：先确认EV8_2(数据真的发出去了)，再发STOP
 * ================================================================
 */
static void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);                          // EV5

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);            // EV6

    I2C_SendData(I2C1, reg);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING);                    // EV8

    I2C_SendData(I2C1, data);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);                     // EV8_2，确认真的发完了

    I2C_GenerateSTOP(I2C1, ENABLE);
}

/**
 * ================================================================
 * [驱动层-内部] MPU6050读寄存器（static，本模块最复杂的部分）
 * ================================================================
 *
 * 六阶段流程：
 *   阶段1：告诉MPU6050"我要操作哪个寄存器"（START、发地址(写)、发reg）
 *   阶段2：换方向，Restart重新起始（再发一次START，再发一次地址，方向改Receiver）
 *   阶段3：因为只收1个字节，提前关闭ACK、提前排队STOP
 *   阶段4：等数据真正到达，再读出来
 *   阶段5：恢复ACK使能（为以后可能的多字节读取做准备）
 *   阶段6：返回读到的值
 *
 * 【为什么需要Restart，曾经完全没意识到】
 *   读寄存器本质是两步：先"写"要读的寄存器地址，再"读"那个寄存器的内容
 *   方向从Transmitter变成Receiver，必须重新发一次START才能切换方向
 *   第一版代码直接从"发完reg"跳到"发接收方向地址"，完全遗漏了这一步
 *
 * 【单字节读取的特殊时序，曾经顺序写反】
 *   I2C硬件要求：ACK/NACK的决定必须在数据字节收完之前就定好
 *   因为只读1个字节，这个字节既是第一个也是最后一个，必须读之前
 *   就提前关闭ACK（让硬件自动回NACK）、提前把STOP排队
 *   第一版代码把I2C_ReceiveData()放在等待EV7之前调用——
 *   在数据还没真正到达时就尝试读取，顺序完全反了
 *
 * 【曾有的疑问：ACK需要手动发送吗？】
 *   不需要。ACK是I2C从机收到字节后自动在硬件层面完成的动作，
 *   不是STM32主动"发送"出去的东西
 *   I2C_CheckEvent()检测的事件本身（如EV6）已经隐含了"ACK已确认"这个条件，
 *   不需要单独写一行代码去处理ACK的产生
 * ================================================================
 */
static uint8_t MPU6050_ReadReg(uint8_t reg)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(I2C1, reg);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING);

    /* Restart：换方向必须重新START，第一版代码遗漏过这一步 */
    I2C_GenerateSTART(I2C1, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Receiver);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    /* 单字节读取特殊时序：必须在等待接收之前就提前处理好 */
    I2C_AcknowledgeConfig(I2C1, DISABLE);  // 提前关闭ACK
    I2C_GenerateSTOP(I2C1, ENABLE);        // 提前申请STOP

    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED);   // 先等数据真正到达(EV7)
    uint8_t Data = I2C_ReceiveData(I2C1);                // 再读出来

    I2C_AcknowledgeConfig(I2C1, ENABLE);   // 恢复ACK，避免影响以后的多字节读取
    return Data;
}

/**
 * ================================================================
 * [驱动层-内部] MPU6050寄存器配置（static，设备层，专属于MPU6050）
 * ================================================================
 *
 * 方法论（贯穿全部六个寄存器的配置原则）：
 *   查手册选项 → 对照硬件实际情况验证 → 对照应用需求验证 → 选择
 *   拒绝"凭感觉"选值，每一位都要有据可查
 *
 * PWR_MGMT_1 = 0x01
 *   SLEEP=0(唤醒) CYCLE=0(不用低功耗周期采样) CLKSEL=001(X轴陀螺仪做时钟参考)
 *   【曾犯的错误】最初"凭感觉"选CYCLE=1(低功耗周期模式，
 *   但平衡车需要持续实时采样，这个模式从应用需求上就不成立)，
 *   以及CLKSEL=101(外部19.2MHz参考，但硬件上根本没有外部时钟源接入)
 *   【曾有的疑问】选X/Y/Z轴哪个做时钟参考，是不是代表"只关心这个方向的姿态"？
 *   答案：完全无关。CLKSEL只是给芯片内部"打节拍"借用哪个陀螺仪谐振结构，
 *   三个轴的测量数据照常全部输出，选哪个轴对最终六轴数据没有任何影响
 *
 * PWR_MGMT_2 = 0x00
 *   六个轴(STBY_XA/YA/ZA/XG/YG/ZG)全部不待机，保持全部轴工作
 *   PWR_MGMT_1是"总电闸"，PWR_MGMT_2是"每个房间的灯开关"，总闸先开，
 *   房间开关才有意义
 *
 * SMPLRT_DIV = 0x04
 *   Sample Rate = 1kHz / (1+4) = 200Hz，匹配IMU读取任务计划频率(200Hz)
 *   【曾犯的错误】最初误以为传感器采样率应该参照电机PWM频率(10kHz)，
 *   两者是完全不相关的物理过程(电力电子开关速度 vs 运动控制采样需求)，
 *   真正该参照的是Task_Manager里IMU读取任务的执行频率
 *
 * CONFIG (DLPF_CFG) = 0x04
 *   带宽21Hz，延迟8.5ms，EXT_SYNC_SET=000(不用FSYNC)
 *   平衡车有电机振动噪声，DLPF必须用较低带宽滤除高频振动，
 *   但带宽太低会导致相位延迟过大、影响控制环响应速度，0x04是噪声抑制
 *   与响应延迟之间的折中
 *
 * GYRO_CONFIG (FS_SEL) = 0x10
 *   FS_SEL=2，±1000°/s
 *   【曾犯的错误】一度误以为可以给X/Y/Z三轴分别配置不同量程，
 *   实际FS_SEL是单一字段，同时控制三个轴的量程，不能分轴设置
 *   最终选±1000°/s留足安全余量，避免电机紧急修正或碰撞产生的瞬时
 *   角速度尖峰超出量程导致数据溢出错乱
 *
 * ACCEL_CONFIG (AFS_SEL) = 0x10
 *   AFS_SEL=2，±8g，ACCEL_HPF=0(不用高通滤波，该功能不影响实际读取的数据寄存器)
 *   平衡车运动中急刹/碰撞可能产生远超过1g静态重力的瞬时加速度，±8g留足余量
 * ================================================================
 */
static void MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);

    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x04);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x04);
    
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);

// MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x10);   // 备选：±1000°/s
// MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x10);  // 备选：±8g
}

/**
 * ================================================================
 * [驱动层] I2C/MPU6050初始化（对外Action接口）
 * ================================================================
 *
 * 职责：封装三个内部初始化函数，对外暴露统一入口
 * 调用方只需要调用这一个函数，不需要知道内部分了协议层/设备层两层
 *
 * 初始化顺序（与Double_Motors_Init先GPIO、再外设参数的顺序保持一致）：
 *   1. I2C_GPIO_Init()      → 引脚准备好
 *   2. I2C1_Protocol_Init() → I2C总线协议就位
 *   3. MPU6050_Init()       → 最后配置MPU6050专属寄存器
 *
 * 【曾经的设计摇摆】
 *   一度考虑把I2C总线初始化和MPU6050寄存器配置直接合并写在一个函数里，
 *   函数名又叫I2C_MPU6050_Init，导致名字和职责范围对不上
 *   最终确认应该拆成三个独立函数、由这一个公开接口统一组装，
 *   理由与Double_Motors_Init()完全一致：
 *   "总线协议初始化"换成任何I2C设备都能复用，"设备寄存器配置"才是MPU6050专属
 * ================================================================
 */
void I2C_MPU6050_Init(void)
{
    I2C_GPIO_Init();
    I2C1_Protocol_Init();
    MPU6050_Init();
}

/**
 * ================================================================
 * [驱动层] 获取MPU6050设备ID（对外Get接口，验证通信用）
 * ================================================================
 *
 * 职责：读WHO_AM_I寄存器（只读，出厂固定值），验证I2C通信链路是否正常
 * 用途：如果接线松动或地址错误，MPU6050_Init()依然能"跑完"
 *       （因为WriteReg只检测ACK这种底层握手），但实际可能根本没配置成功
 *       GetID提供一个明确的验证手段：读出来的值应等于手册标明的固定值
 * ================================================================
 */
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}



/**
 * ================================================================
 * [驱动层] 获取六轴原始数据（对外Get接口）
 * ================================================================
 *
 * 职责：读取加速度计/陀螺仪共6个轴，每轴16位(H+L两个寄存器拼接)
 *
 * 【曾犯的错误：单行写法的求值顺序歧义 + I2C时序风险】
 *   最初尝试写成一行：
 *     Data->GyroX = (ReadReg(H) << 8) | ReadReg(L);
 *   C语言不保证函数参数的求值顺序，理论上H和L的实际读取顺序无法确定
 *   而且H和L是两次完全独立的I2C事务，顺序被打乱或两次读取之间寄存器
 *   又刷新了一次，可能读到"半新半旧"的数据
 *   改正：拆成两步，显式存入临时变量，再拼接，杜绝顺序歧义
 *
 * 【曾经的过度设计：把中间值暴露成了接口】
 *   一度定义了MPU6050_Info_H / MPU6050_Info_L两个额外结构体，
 *   把"高字节""低字节"这种纯粹的拼接中间值做成了函数参数
 *   调用者根本不需要、也不该知道这些实现细节
 *   改正：Data_H/Data_L只是函数内部反复复用的局部变量，读完一轴
 *   立刻被下一轴覆盖，不污染对外接口——这是"接口只暴露调用者
 *   真正需要的东西"原则的具体应用
 *
 * 【局部变量类型，曾反复出错】
 *   Data_H/Data_L最终必须是uint8_t（理由见MPU6050_Data_Merge的说明）
 * ================================================================
 */


void MPU6050_GetData(MPU6050_Info* Data)
{
    uint8_t buf[14];
    
    // 发送起始寄存器地址0x3B（AccX_H）
    I2C_GenerateSTART(I2C1, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);
    
    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    
    I2C_SendData(I2C1, 0x3B);  // AccX_H起始地址
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    
    // Restart，切换为接收模式
    I2C_GenerateSTART(I2C1, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);
    
    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Receiver);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    
    // 连续读14个字节
    for(int i = 0; i < 14; i++)
    {
        if(i == 13)  // 最后一个字节前关闭ACK
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED);
        buf[i] = I2C_ReceiveData(I2C1);
    }
    
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    
    // 拼接数据
    Data->AccX  = (int16_t)(buf[0]  << 8 | buf[1]);
    Data->AccY  = (int16_t)(buf[2]  << 8 | buf[3]);
    Data->AccZ  = (int16_t)(buf[4]  << 8 | buf[5]);
    // buf[6]/buf[7] 是温度，跳过
    Data->GyroX = (int16_t)(buf[8]  << 8 | buf[9]);
    Data->GyroY = (int16_t)(buf[10] << 8 | buf[11]);
    Data->GyroZ = (int16_t)(buf[12] << 8 | buf[13]);
}

/*旧设计：字节分离读取*/
// void MPU6050_GetData(MPU6050_Info* Data)
// {
//     uint8_t Data_H, Data_L;

//     Data_H = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
//     Data_L = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
//     Data->GyroX = MPU6050_Data_Merge(Data_H, Data_L);

//     Data_H = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
//     Data_L = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
//     Data->GyroY = MPU6050_Data_Merge(Data_H, Data_L);

//     Data_H = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
//     Data_L = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
//     Data->GyroZ = MPU6050_Data_Merge(Data_H, Data_L);

//     Data_H = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
//     Data_L = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
//     Data->AccX = MPU6050_Data_Merge(Data_H, Data_L);

//     Data_H = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
//     Data_L = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
//     Data->AccY = MPU6050_Data_Merge(Data_H, Data_L);

//     Data_H = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
//     Data_L = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
//     Data->AccZ = MPU6050_Data_Merge(Data_H, Data_L);
// }


/**
 * ================================================================
 * [驱动层-内部] 高低字节拼接（static）
 * ================================================================
 *
 * 职责：把两个8位寄存器值拼接成一个有符号的16位数
 *
 * 【核心原则：字节本身没有符号，只有拼接后的整体才有】
 *   单个字节只是"原始比特片段"，提前赋予符号会在拼接(<<、|)过程中
 *   发生符号扩展，污染本不该受影响的高位部分
 *   例：若低字节0xFF被当作int8_t(-1)参与运算，符号扩展会把高字节的
 *   信息整个吃掉，结果完全错误
 *   正确做法：参数和中间存储全程保持uint8_t，只让最终拼接结果
 *   被解读为int16_t——符号只应该是"整体"的属性，不该提前出现在"半成品"上
 *
 * 【曾犯的错误，连续两轮】
 *   返回类型第一版是int16_t(正确)，
 *   后来在另一次重写中误改成int8_t——16位拼接结果被硬塞进8位空间，
 *   高位信息直接被截断丢失，这是"看似改对了一处、实则引入新错误"的典型案例
 *   提醒：每次修改后要整体复查，不能只盯着改的那一行
 * ================================================================
 */
// static int16_t MPU6050_Data_Merge(int16_t High, int16_t Low)
// {
//     return High << 8 | Low;
// }

