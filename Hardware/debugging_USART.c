#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

/**
 * ================================================================
 * 【五层塔定位】调试基础设施 - 串口调试模块
 * ================================================================
 *
 * 本模块的特殊性：
 *   调试串口不属于五层塔的任何一层业务逻辑，
 *   它是贯穿所有层的"观测窗口"——每一层都可以用它输出数据。
 *   类比：脚手架，不是楼的一部分，但建楼离不开它。
 *
 *   监控层  → 用串口打印报警信息、低压警告
 *   调度层  → 每100ms发送一次数据（后台任务）
 *   算法层  → 打印PID中间变量，辅助调参
 *   反馈处理层→ 打印原始IMU数据、滤波后角度
 *   驱动层  ← 本模块，配置USART2硬件
 *
 * 函数分层结构（调用依赖关系）：
 *   Printf
 *     └→ SendString
 *          └→ SendBytes  ← 真正的底层，直接操作硬件
 *               └→ SendByte
 *   report_error（内部错误报告，static封装，只用SendByte）
 *
 * 对外接口（.h文件声明）：
 *   Debugging_USART_Init()      → 初始化硬件
 *   Debugging_USART_SendByte()  → 发送一个字节
 *   Debugging_USART_SendBytes() → 发送多个字节
 *   Debugging_USART_SendString()→ 发送字符串
 *   Debugging_USART_ReceiveByte() → 接收一个字节
 *   Debugging_USART_ReceiveBytes()→ 接收多个字节
 *
 * 【为什么不用fputc重定向printf？】
 *   fputc重定向把printf绑定到USART，全局生效，不够灵活。
 *   自己实现SendString/Printf，可以明确控制发到哪个串口，
 *   也避免了链接stdio库带来的代码体积增加。
 *
 * 【调试通道是开发的第一步】
 *   没有串口，所有传感器数据都是黑箱。
 *   开发顺序：串口调试通道 → 传感器 → 控制算法
 * ================================================================
 */

/* ----------------------------------------------------------------
 * 前向声明（因为report_error在文件顶部定义，但调用了SendByte）
 *
 * 【曾有的疑问】
 *   Q: 为什么需要前向声明？
 *   A: C编译器从上往下读，如果report_error调用了SendByte，
 *      但SendByte定义在report_error后面，编译器看到调用时
 *      还不认识SendByte，会报错。
 *      前向声明告诉编译器："这个函数存在，后面会定义"
 * ---------------------------------------------------------------- */
static void report_error(const char *msg);
void Debugging_USART_SendByte(uint8_t info);

/**
 * ================================================================
 * [驱动层] 调试串口初始化
 * ================================================================
 *
 * 职责：
 *   激活USART2硬件，为上层提供调试数据收发能力。
 *   本函数是调试通道的唯一硬件入口，上层无需关心引脚和外设细节。
 *
 * 硬件约束（来自原理图）：
 *   PA2 → USART2_TX（发送）
 *   PA3 → USART2_RX（接收）
 *   USART2挂载于APB1总线
 *   系统时钟72MHz
 *
 * 通信参数（115200 8N1）：
 *   波特率 115200 → 调试串口标准选择，几乎所有工具都支持
 *   数据位 8bit   → 标准字节传输
 *   校验位 无(N)  → 调试场景无需校验开销
 *   停止位 1bit(1)→ 告诉接收方帧结束，拉高电平至空闲状态
 *
 * 【曾犯的错误】
 *   GPIO结构体覆盖问题：
 *   TX和RX配置放在同一个结构体，只在最后调用一次GPIO_Init，
 *   导致TX的配置被RX覆盖，最终只有RX的配置生效。
 *   正确做法：配置TX → 调用GPIO_Init → 配置RX → 再调用GPIO_Init
 *
 *   函数声明缺少void参数：
 *   void Debugging_USART_Init() → 产生deprecated declaration警告
 *   正确：void Debugging_USART_Init(void)
 *
 * 【曾有的疑问】
 *   Q: TX为什么是复用推挽输出，RX为什么是上拉输入？
 *   A: TX发送数据，需要主动驱动引脚高低电平 → 推挽输出
 *      复用：引脚由USART2外设控制，不是GPIO寄存器直接控制
 *      RX接收数据，需要读取外部信号 → 输入模式
 *      上拉：串口空闲状态是高电平，上拉保持一致，避免浮空误触发
 *      浮空输入在无信号时电平不确定，容易产生乱码
 *
 *   Q: 停止位的作用是什么？
 *   A: 告诉接收方"这一帧数据结束了，准备接收下一帧"
 *      没有停止位，接收方不知道帧边界在哪里，数据会乱
 *
 *   Q: TX接TX，RX接RX可以吗？
 *   A: 不行，必须交叉连接：
 *      CH340的TX（发出去）→ STM32的RX（PA3，接收）
 *      CH340的RX（接进来）→ STM32的TX（PA2，发送）
 *      实际调试时就犯过这个错误，导致什么都收不到
 *
 *   Q: GND为什么要共地？
 *   A: 电压是相对值，必须有公共参考点（GND）
 *      面包板两侧电源轨可能不连通，需要用跳线把两侧GND连接起来
 * ================================================================
 */
void Debugging_USART_Init(void)
{
    /* [1] 开启外设时钟
     * USART2 → APB1总线（低速，最高36MHz）
     * GPIOA  → APB2总线（高速，PA2/PA3均属于GPIOA）
     * 时钟不开，外设寄存器写入无效 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    /* [2] PA2配置为复用推挽输出（TX）
     * 必须先Init TX，再Init RX，不能共用一次Init
     * 否则RX的配置会覆盖TX的配置 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);      // TX单独Init一次

    /* [3] PA3配置为上拉输入（RX）
     * 上拉：空闲时保持高电平，与串口空闲状态一致
     * 不用浮空：浮空在无信号时电平不确定，容易误触发 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);      // RX单独Init一次

    /* [4] 配置USART2通信参数（115200 8N1） */
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    /* [5] 使能USART2，硬件开始工作 */
    USART_Cmd(USART2, ENABLE);
}

/**
 * ================================================================
 * [内部] 错误报告函数（static，不对外暴露）
 * ================================================================
 *
 * 【为什么用static？】
 *   这个函数只在本文件内部使用，外部不需要知道它的存在。
 *   static限制可见范围在本文件内，是封装的体现。
 *
 * 【为什么只用SendByte，不用SendString？】
 *   防御性编程：避免递归。
 *   如果report_error调用SendString，SendString内部又调用report_error，
 *   会造成无限递归→栈溢出→程序崩溃。
 *   错误处理函数本身必须足够简单，只依赖最底层函数。
 *
 * 【曾有的疑问】
 *   Q: *msg++和msg++有什么区别？
 *   A: *msg是解引用（取当前字符的值），msg++是移动指针
 *      我们要移动的是指针本身，所以用msg++
 *      *msg++会先解引用再移动，语义上也能工作，但不够清晰
 * ================================================================
 */
static void report_error(const char *msg)
{
    while (*msg != '\0')       // 遇到字符串结束符停止
    {
        Debugging_USART_SendByte(*msg);  // 只依赖最底层函数，避免递归
        msg++;                 // 指针移到下一个字符
    }
}

/**
 * ================================================================
 * [驱动层] 发送一个字节（最底层发送函数）
 * ================================================================
 *
 * 所有上层发送函数最终都调用这里。
 *
 * 发送流程：
 *   等待TXE（发送缓冲区空）→ 写入数据寄存器 → 硬件自动发送
 *
 * STM32发送有两个寄存器：
 *   TDR（发送数据寄存器）→ 你能访问的，写入数据
 *   发送移位寄存器        → 硬件自动操作，一位位发出去
 *
 * 标志位：
 *   TXE  = 发送数据寄存器空（可以写下一个字节）
 *   TC   = 发送完成（移位寄存器也空了，线上没有数据）
 *   RXNE = 接收数据寄存器非空（有数据可以读）
 *
 * 【曾犯的错误】
 *   等待条件写反：while(TXE == SET)应为while(TXE == RESET)
 *   口诀：等到"好了"才动手
 *         TXE==RESET → 寄存器还满着，继续等
 *         TXE==SET   → 寄存器空了，可以写了
 *
 *   标志位用错：误用RXNE（接收标志）代替TXE（发送标志）
 *
 * 【曾有的疑问】
 *   Q: 参数为什么是uint8_t而不是char？
 *   A: char在不同编译器下有无符号/有符号歧义
 *      uint8_t明确表示"无符号8位字节"，语义清晰
 *      接口参数类型应跟着语义走，不跟底层硬件寄存器宽度走
 *      （USART_SendData接受uint16_t是因为寄存器是16位，
 *       但我们发送的语义是"一个字节"，所以接口用uint8_t）
 * ================================================================
 */
void Debugging_USART_SendByte(uint8_t info)
{
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, info);
}

/**
 * ================================================================
 * [驱动层] 发送多个字节
 * ================================================================
 *
 * 上层函数的基础，SendString和Printf最终都调用这里。
 *
 * 【防御性编程】
 *   检查两个最危险的参数：
 *   array == NULL → 空指针，访问会崩溃
 *   size == 0    → 没有数据要发，循环没有意义
 *   出错时通过report_error报告，而不是静默失败
 *
 * 【曾有的疑问】
 *   Q: 为什么不在SendByte里直接操作，而要有SendBytes这一层？
 *   A: 分层设计，SendBytes负责"循环逻辑"，SendByte负责"硬件操作"
 *      上层改变不影响下层，下层改变（如换成DMA）不影响上层
 * ================================================================
 */
void Debugging_USART_SendBytes(uint8_t *array, uint16_t size)
{
    if (array == NULL || size == 0)
    {
        report_error("[ERROR] SendBytes: invalid args\n");
        return;
    }

    for (int i = 0; i < size; i++)
    {
        Debugging_USART_SendByte(array[i]);
    }
}

/**
 * ================================================================
 * [驱动层] 发送字符串
 * ================================================================
 *
 * C语言字符串本质：char数组 + '\0'结束符
 * strlen计算字符数量（不含'\0'），SendBytes按字节发送
 *
 * 【曾有的疑问】
 *   Q: 为什么需要(uint8_t *)强制转换？
 *   A: char*和uint8_t*底层都是1字节，可以互转
 *      这里只是告诉编译器"我知道类型不完全匹配，但没问题"
 *      char* = 我在处理文本，uint8_t* = 我在处理原始字节
 *      两者底层一模一样，只是语义标签不同
 *
 *   Q: 数字（float/uint16_t）可以直接用SendString发送吗？
 *   A: 不能。数字在内存里是二进制，人看不懂。
 *      必须先用sprintf转换成字符串，再用SendString发出去：
 *      float 4.22 → sprintf → "4.22V\n" → SendString → 串口
 *      sprintf = 格式化转换器，不是打印，结果存在char数组里
 *
 * 【防御性编程】
 *   检查NULL和空字符串，防止无效调用
 * ================================================================
 */
void Debugging_USART_SendString(const char *string)
{
    if (string == NULL || strlen(string) == 0)
    {
        report_error("[ERROR] SendString: invalid args\n");
        return;
    }

    Debugging_USART_SendBytes((uint8_t *)string, strlen(string));
}

/**
 * ================================================================
 * [驱动层] 接收一个字节（最底层接收函数）
 * ================================================================
 *
 * 接收流程：
 *   等待RXNE（接收寄存器非空）→ 读取数据寄存器
 *
 * 【曾犯的错误】
 *   等待条件写反：while(RXNE == SET)应为while(RXNE == RESET)
 *   口诀：等到"好了"才动手
 *         RXNE==RESET → 寄存器还空着，没有数据，继续等
 *         RXNE==SET   → 有数据了，可以读了
 *
 * 【局限性】
 *   这是阻塞式接收，会一直等到有数据为止。
 *   如果对方不发数据，程序卡死。
 *   完整实现应加Timeout超时保护，需要依赖TIM3时基（调度层）。
 *   目前TIM3时基已建立，可以后续补充带超时的版本。
 * ================================================================
 */
uint8_t Debugging_USART_Receivebyte(void)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
    return (uint8_t)USART_ReceiveData(USART2);
}

/**
 * ================================================================
 * [驱动层] 接收多个字节
 * ================================================================
 *
 * 【曾有的疑问】
 *   Q: 为什么参数是指针而不是返回数组？
 *   A: C函数只有一个返回值，无法返回数组。
 *      通过传入指针，函数直接往调用方的内存里写数据，
 *      调用方就能拿到所有接收到的字节。
 *      这是C语言实现"多返回值"的标准做法。
 *      类比CODESYS里的VAR_IN_OUT传引用，本质相同。
 *
 * 【防御性编程】
 *   检查NULL指针和size==0，防止无效调用
 *
 * 【待完善】
 *   缺少Timeout参数——当前版本如果对方不发数据会卡死。
 *   后续可参考铁头山羊原版实现，依赖System_GetTick()添加超时。
 * ================================================================
 */
void Debugging_USART_Receivebytes(uint8_t *array, uint16_t size)
{
    if (array == NULL || size == 0)
    {
        report_error("[ERROR] ReceiveBytes: invalid args\n");
        return;
    }

    for (int i = 0; i < size; i++)
    {
        array[i] = Debugging_USART_Receivebyte();
    }
}

