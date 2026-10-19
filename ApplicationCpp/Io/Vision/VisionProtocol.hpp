/* ============================================================================
 * 文件：VisionProtocol.hpp
 * 作用：视觉上位机串口协议的“纯逻辑”层——消帧（0xA5 帧头 / 0x5A 帧尾）与
 *       解码（线上字节结构 → C++ 领域快照）。
 * 约束：本层不允许依赖 HAL、FreeRTOS、USB 等任何硬件/OS 头文件，
 *       从而可以拿录制下来的字节流在 PC 上做回放测试。
 * 依赖：仅 <cstdint>。
 * ========================================================================== */
#ifndef robot_io_vision_protocol_hpp
#define robot_io_vision_protocol_hpp

#include <cstdint>

namespace robot::io::vision {

// 帧边界（与 Application/inc/Vision.h 的 VISION_FRAME_HEADER_RX / END_RX 一致）。
inline constexpr std::uint8_t k_frame_header = 0xA5U; // 接收帧头
inline constexpr std::uint8_t k_frame_end = 0x5AU;    // 接收帧尾

// mode 字段取值（与 Vision.h 的 VISION_RECEIVE_* 枚举一致）。
inline constexpr std::uint8_t k_mode_none = 0U;
inline constexpr std::uint8_t k_mode_armor = 1U;
inline constexpr std::uint8_t k_mode_rune = 2U;

// 视觉接收帧的线上布局：历史上逐字节对应 Vision.h 的 VisionReceive（该类型已删除）。
// 这里刻意重新声明而不 include Vision.h：Vision.h 会带入 main.h / Shooter.h
// 等硬件与业务头，破坏本层的可移植性。布局由下方 static_assert 锁定。
#pragma pack(push, 1)
struct vision_receive_wire {
    std::uint8_t header;
    std::uint8_t mode;
    union {
        struct {
            float base_yaw;
            float yaw;
            float pitch;
            float incident_yaw;
            float distance;
            float hit_radius;
        } armor; // 打装甲板模式
        struct {
            float base_yaw;
            float hit_radius;
            float yaw[2];
            float pitch[2];
            float incident_yaw[2];
            float distance[2];
            float hit_time[2];
            std::int8_t id[2];
        } rune; // 打符模式
    };
    std::uint8_t force_shoot; // 视觉端强制开火请求
    float linear_x;           // AI 底盘线速度
    float linear_y;
    float angular_z; // 旋转速度
    float align_yaw; // 起伏路段对齐角
    float look_yaw;  // 符/前哨站注视角
    float look_pitch;
    std::uint8_t spin_mode;
    std::uint8_t sentry_mode;
    std::uint8_t armor_mode;
    std::uint8_t align_mode;
    std::uint8_t energy_activation;
    std::uint8_t buy_life;
    std::uint8_t remote_buy_blood;
    std::uint8_t remote_buy_bullet;
    std::uint16_t buy_projectile;
    std::uint8_t end_frame;
};
#pragma pack(pop)

static_assert(sizeof(vision_receive_wire) == 88U,
              "vision_receive_wire 必须保持 88 字节（视觉接收协议）");

// 一帧视觉数据解码后的领域快照（平凡可复制，可直接放进快照邮箱）。
// 语义约定：
//   - 角度单位统一为“度”，与旧全局 vision_target 完全一致；
//   - has_target 对应旧字段 vaild（有没有目标）；链路新鲜度不在这里表达，
//     消费者应结合快照邮箱的 age/TTL 自行判断；
//   - id_to_trigger 为 -1 表示“无触发目标”（旧结构未设置时会残留旧值，
//     这里改成显式 -1，避免歧义）。
struct vision_frame_snapshot {
    bool has_target;           // 本帧是否存在可跟踪目标。
    std::uint8_t raw_mode;     // 线上 mode 原值（none/armor/rune）。
    std::int8_t id_to_trigger; // 待触发目标 id，-1 表示无。
    bool force_shoot;          // 视觉端强制开火请求。
    float base_yaw;            // 目标角度，度。
    float yaw;                 // 目标角度，度。
    float pitch;               // 目标角度，度。

    // —— 打装甲板判定所需的原始字段（旧 Vision_UseArmor 使用）——
    float armor_incident_yaw; // 装甲板入射角，度。
    float armor_distance;     // 目标距离。
    float armor_hit_radius;   // 命中半径。

    // —— 打符判定所需的原始字段（旧 Vision_UseRune 使用）——
    float rune_yaw[2];          // 两个符的角度，度。
    float rune_pitch[2];        // 两个符的 pitch，度。
    float rune_incident_yaw[2]; // 两个符的入射角，度。
    float rune_distance[2];     // 两个符的距离。
    float rune_hit_time[2];     // 两个符的命中延时，秒。
    std::int8_t rune_id[2];     // 两个符的 id，-1 表示无效。
    float rune_hit_radius;      // 符的命中半径。

    float linear_x;            // AI 底盘线速度指令（视觉坐标系）。
    float linear_y;
    float angular_z;
    float align_yaw; // 起伏路段对齐角，度。
    float look_yaw;  // 符/前哨站注视角，度。
    float look_pitch;
    std::uint8_t spin_mode;
    std::uint8_t sentry_mode;
    std::uint8_t armor_mode;
    std::uint8_t align_mode;
    std::uint8_t energy_activation;
    std::uint8_t buy_life;
    std::uint8_t remote_buy_blood;
    std::uint8_t remote_buy_bullet;
    std::uint16_t buy_projectile;
};

// 快照会放进 latest_snapshot_mailbox（上限 128B），这里提前锁定尺寸。
static_assert(sizeof(vision_frame_snapshot) <= 128U,
              "vision_frame_snapshot 超出快照邮箱的固定容量");

// 把一次完整线上帧解码成领域快照（纯函数，无副作用）。
vision_frame_snapshot decode_vision_frame(const vision_receive_wire& wire);
// 字节流消帧器：状态只有“正在填充的帧 + 已填字节数”，无堆分配。
// 用法：把每次收到的字节块喂给 feed()；返回 true 表示本次调用中至少解出
// 一帧，产生的最后一帧通过 last_frame() 取用（同一块里出现多帧时取最新，
// 中间帧可丢弃——控制侧只关心最新目标）。
class vision_frame_parser {
public:
    // 喂入一段字节流；返回 true 表示本次调用中至少解出一帧。
    bool feed(const std::uint8_t* data, std::uint32_t length);

    // 最近一次成功解码的帧（feed() 返回 true 后有效）。
    const vision_frame_snapshot& last_frame() const { return last_frame_; }

    // 累计解出的完整帧数量（诊断/旁路对比用）。
    std::uint32_t frame_count() const { return frame_count_; }

    // 复位消帧状态（重新同步时使用）。
    void reset()
    {
        wire_ = vision_receive_wire{};
        fill_index_ = -1;
        last_frame_ = vision_frame_snapshot{};
        frame_count_ = 0U;
    }

private:
    vision_receive_wire wire_{};         // 正在填充的线上帧。
    std::int32_t fill_index_{-1};        // -1 表示未锁定帧头；否则为已填字节数。
    vision_frame_snapshot last_frame_{}; // 最近一次解码结果。
    std::uint32_t frame_count_{0U};      // 累计完整帧数。
};

} // namespace robot::io::vision

#endif // robot_io_vision_protocol_hpp
