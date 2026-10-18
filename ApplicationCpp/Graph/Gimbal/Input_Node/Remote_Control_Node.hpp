#ifndef _REMOTE_CONTROL_NODE_HPP_
#define _REMOTE_CONTROL_NODE_HPP_

#include "PluginNode.hpp"
#include "PluginTypes.hpp"
#include "Ports.hpp"

#define VISION_FRAME_HEADER_TX   0x5A   // vision上位机发送帧头
#define VISION_FRAME_END_TX      0xA5   // vision上位机发送帧尾
#define VISION_FRAME_HEADER_RX   0xA5   // vision上位机接收帧头
#define VISION_FRAME_END_RX      0x5A   // vision上位机接收帧尾

enum { VISION_RECEIVE_NONE, VISION_RECEIVE_ARMOR, VISION_RECEIVE_RUNE }; // 上位机接收状态
enum { VISION_AI_ARMOR, VISION_AI_OUTPOST, VISION_AI_RUNE }; // 上位机ai模式

// region 上位机传来的数据类型
typedef struct __attribute__((packed))
{
    uint8_t header;
    /*以下为视觉通信部分*/
    /*打装甲板模式*/
    uint8_t mode;

    union {
        struct __attribute__((packed)) {
            float base_yaw;
            float yaw;
            float pitch;
            float incident_yaw;
            float distance;
            float hit_radius;
        } armor;// 打装甲板

        struct __attribute__((packed)) {
            float base_yaw;
            float hit_radius;
            float yaw[2];
            float pitch[2];
            float incident_yaw[2];
            float distance[2];
            float hit_time[2];
            int8_t id[2];
        } rune;// 打符
    };

    uint8_t force_shoot;//检测到 强制开一发火 0是不开 1为开火
    
/***********以下为ai传输内容***********/
    float linear_x;
    float linear_y;     
    float angular_z; //旋转速度
    
    float align_yaw; //和起伏路段对齐角度
    float look_yaw; //符或前哨站的角度
    float look_pitch;

    uint8_t spin_mode;  //0为小陀螺关  1为小陀螺开
    uint8_t sentry_mode; //1为进攻 2为防守 3为移动 默认为3
    uint8_t armor_mode;  //0为打车 1打前哨 2为打符
    uint8_t align_mode;  //是否对齐 0为不对齐 1为对齐装甲板
    uint8_t energy_activation; //0为不激活 1为激活小符 2为激活大符 激活小还是大和比赛开始时间有关
    uint8_t buy_life;     //0为不买活 1为买活
    uint8_t remote_buy_blood;    //0到1为远程买一次血 1到2为买一次 依此类推 
    uint8_t remote_buy_bullet;  //0到1为远程买一次弹 1到2为买一次 依此类推
    uint16_t buy_projectile; //哨兵要买多少发弹 开局为0 修改后烧饼在补血点就能兑换 只能单增 如0->100买100发 100->101买1发 依此类推
    uint8_t end_frame;   
}VisionReceive;
// endregion

// 视觉目标
typedef struct {
    bool vaild;
    float base_yaw;
    float yaw;
    float pitch;
    int8_t idToTrigger;
    uint8_t shooterCommand;
} VisionTarget;

// region Gimbal节点
namespace robot::Gimbal
{
    class Remote_Control_Node final : public robot::framework::plugin_node
    {
    public:
        explicit Remote_Control_Node(robot::framework::plugin_id id_) : plugin_node(id_) {}

        void bind_vision(const robot::framework::frame_signal<float> source)
        {
            vision_ = &source; 
        }

        const robot::framework::frame_signal<float>& output() const
        {
            return output_;
        }

        // 执行代码
        robot::framework::process_result process(robot::framework::frame_context& context) override;

        // 跳过
        void on_skipped(robot::framework::frame_context& context) override
        {
            (void)context;
            output_.invalidate();
        }


    private:
        const robot::framework::frame_signal<float>* vision_{nullptr};
        robot::framework::frame_signal<float> output_{};
    };
}
// endregion

#endif