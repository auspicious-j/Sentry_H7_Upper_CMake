#ifndef _REMOTE_CONTROL_NODE_HPP_
#define _REMOTE_CONTROL_NODE_HPP_

#include "PluginNode.hpp"
#include "PluginTypes.hpp"
#include "Ports.hpp"

// 视觉协议类型（VisionReceive / VisionTarget 及其宏与枚举）统一由
// Application/inc/Vision.h 提供，节点头文件不再复制一份，避免同一类型
// 在不同编译单元出现两份定义（ODR 冲突）.

// region Gimbal节点
namespace robot::Gimbal
{
    class Remote_Control_Node final : public robot::framework::plugin_node
    {
    public:
        explicit Remote_Control_Node(robot::framework::plugin_id id_) : plugin_node(id_) {}

        // 组装期（图冻结前）绑定上游信号。必须按引用传参：按值传参会把
        // 函数内副本的地址存进 vision_，函数返回后即悬空。
        void bind_vision(const robot::framework::frame_signal<float>& source)
        {
            vision_ = &source;
        }

        const robot::framework::frame_signal<float>& output() const
        {
            return output_;
        }

        // 执行代码（定义在 Remote_Control_Node.cpp）
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