// ============================================================================
// Beep.c —— 蜂鸣器(无源蜂鸣器 PWM 驱动)模块实现
// ----------------------------------------------------------------------------
// 模块用途：用 TIM12 通道 2 输出 PWM 方波驱动蜂鸣器，通过改变方波周期(音高)和
//   持续时间(时值)来播放简单的乐曲/提示音。
// 数据流：调用方 Beep_PlayNotes() 把音符入队 -> 唤醒蜂鸣器任务 -> 任务的回调
//   Task_Beep_Callback() 每次取出一个音符，设置 PWM 频率并延时对应时值。
// 并发说明：入队方(任意任务/中断)与出队方(蜂鸣器任务)通过 FreeRTOS 队列 +
//   任务通知配合；队列操作本身有临界区保护，细节见 myQueue.h。
// ============================================================================
#include "Beep.h"
#include "myQueue.h"
#include "UserFreertos.h"
#include "tim.h"

// 软件忙等延时宏：SOFTWARE_DELAY(t) 延时约 t 毫秒(t=0 则不延时)。
// 实现：int i = t*12000，然后 while(i--) 空转递减到 0。
// 用途：在调度器未运行或中断上下文中使用(此时不能调用 osDelay 这类会阻塞/切换任务的函数)。
// 注意：它是“空转”延时，会占满 CPU；系数 12000 是按当前主频粗略标定的，仅用于蜂鸣器节拍。
#define SOFTWARE_DELAY(t) do{uint32_t i=(t)*12000; while(i--);}while(0)//软件延时

// 音符队列的最大容量(可同时缓存的音符个数)。
// 队列满时再入队会失败/覆盖，故连续调用 Beep_PlayNotes 时音符数不要超过该值。
#define MAX_NOTE_QUEUE_SIZE 50

Queue noteQueue;// 音符队列对象：存放“将要播放”的音符(FIFO，先入先出)
Note noteQueueBuf[MAX_NOTE_QUEUE_SIZE];// 队列的底层存储数组，元素为 Note，由 Queue_AttachBuffer 绑定给 noteQueue

// 蜂鸣器初始化：上电后调用一次。
// 1) 先启动 TIM12 通道 2 的 PWM 输出(此时尚未设定频率，具体音高由 Beep_Play 决定)；
// 2) 初始化音符队列，并把静态缓冲区 noteQueueBuf 挂接给它(队列自身不分配内存)。
// 参数：无
// 返回值：无
void Beep_Init()
{
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
	//队列初始化
    Queue_Init(&noteQueue,MAX_NOTE_QUEUE_SIZE);
    Queue_AttachBuffer(&noteQueue,noteQueueBuf,sizeof(Note));
}

// 向蜂鸣器输出指定周期(us)、占空比 50% 的方波(内部辅助函数，未在头文件导出)。
// 参数：period —— 方波周期，单位 us(即 1/频率)；传入 0 表示静音(停止输出)。
// 返回值：无
void Beep_Play(uint16_t period)
{
    HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_2);  //先停波，避免修改定时器参数时产生杂波
    if(period!=0)  //周期 0 视为静音：只停波、不重新启动
	{
    __HAL_TIM_SetAutoreload(&htim12,period);  //ARR=周期，决定方波频率
    __HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_2,period/2);  //CCR=周期的一半，得到 50% 占空比
	    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);  //重新启动 PWM 输出
	}
}

// 播放一个音符(非阻塞)：把音符拷贝进队列，并唤醒蜂鸣器任务去处理。
// 参数：note —— 指向待播放音符；函数会把其内容拷贝入队，调用后 note 可释放/复用。
// 返回值：无
// 注意：队列满时 Queue_Enqueue 的行为取决于 myQueue 实现(可能失败或覆盖最旧元素)。
void Beep_PlayOneNote(Note *note)
{
	//入队
    Queue_Enqueue(&noteQueue,note);
    xTaskNotifyGive(BeepTaskHandle);  //通知beep任务开始
}

// 播放一串音符(非阻塞)：把 notes[0..noteNum-1] 依次入队，再唤醒蜂鸣器任务。
// 参数：notes —— 音符数组首地址；noteNum —— 音符个数(不应超过队列容量 MAX_NOTE_QUEUE_SIZE)
// 返回值：无
void Beep_PlayNotes(Note notes[],uint8_t noteNum)
{
	//入队
    for(int i=0;i<noteNum;i++)
	    Queue_Enqueue(&noteQueue,&notes[i]);
    xTaskNotifyGive(BeepTaskHandle);
}

// 阻塞式播放队列中全部音符：一直等到队列空、全部播完才返回。
// 参数：无
// 返回值：无
// 警告：内部用软件忙等 + 顺序执行，会独占 CPU，不应在任务调度正常运行时调用
//       (可在调度器启动前或中断里用来播放启动音)。
void Beep_Block_PlayAll()
{
    while(!Queue_IsEmpty(&noteQueue))//播放队列中所有音符
	{
	    Note *note=Queue_Dequeue(&noteQueue);  //取出队头音符
	    Beep_Play(note->period);  //按该音符的周期发声
	    SOFTWARE_DELAY(note->duration);//阻塞式延时，使用软件延时防止在中断中调用
	}
    Beep_Play(T_None);  //播完后停声，避免蜂鸣器一直响
}

// 蜂鸣器任务回调：由 FreeRTOS 蜂鸣器任务周期调用(每次只处理一个音符)。
// 逻辑：队列空 -> 停声并挂起等待任务通知；队列非空 -> 取一个音符发声，并延时其时值。
// 参数：无
// 返回值：无
void Task_Beep_Callback()
{
    if(Queue_IsEmpty(&noteQueue))//没有音符要播放则停止任务
	{
	    Beep_Play(T_None);  //静音
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //等待任务通知
	}
    else
	{
		//播放队头音符并等待指定时值
	    Note *note=Queue_Dequeue(&noteQueue);  //取出队头音符
	    Beep_Play(note->period);  //按该音符周期发声
        osDelay(note->duration);  //在任务中可安全使用 osDelay 阻塞到时值结束
	}
}

/************************freertos任务***************************/
// 蜂鸣器 FreeRTOS 任务入口函数：任务启动后循环执行。
// 参数：argument —— 创建任务时传入的参数(本函数未使用)
// 返回值：无(for(;;) 常驻循环，理论上不会返回)
void OS_BeepCallback(void const * argument)
{
  //蜂鸣器播放启动声 [do re me so la(升)]
   // Beep_PlayNotes((Note[]){{T_M1,D_Sixteenth},{T_M3,D_Sixteenth},{T_M5,D_Sixteenth},{T_H1,D_Sixteenth},{T_None,D_Quarter}},5);
    for(;;)
	{
       // Task_Beep_Callback();
        osDelay(20);  //当前未启用回调，仅每 20ms 空转一次(播放任务调用的代码已注释)
	}
}

