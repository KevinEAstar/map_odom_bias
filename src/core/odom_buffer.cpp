/**
 * @file odom_buffer.cpp
 * @brief OdomBuffer 实现: 环形缓冲 + 插值 + 边界 + reset 清空/静默
 */

#include "map_odom_bias/core/odom_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace map_odom_bias
{

namespace
{

bool sample_is_finite(const OdomSample & s)
{
    return std::isfinite(s.t.s) &&
           std::isfinite(s.pose.p[0]) && std::isfinite(s.pose.p[1]) &&
           std::isfinite(s.pose.p[2]) &&
           std::isfinite(s.pose.q.w) && std::isfinite(s.pose.q.x) &&
           std::isfinite(s.pose.q.y) && std::isfinite(s.pose.q.z);
}

}  // namespace

OdomBuffer::OdomBuffer(double duration, double max_extrapolation)
: duration_(duration), max_extrapolation_(max_extrapolation)
{
}

bool OdomBuffer::push(const OdomSample & sample)
{
    // 有限性守卫: 单个 NaN/inf 样本会毒化整个插值窗口并破坏缓冲有序性
    if (!sample_is_finite(sample)) {
        ++invalid_drop_count_;
        return false;
    }
    // reset 静默窗口: 旧坐标系的乱序尾巴直接丢弃 (详设 4.6 附带动作 3)
    if (sample.t.s < settle_until_.s) {
        ++settle_drop_count_;
        return false;
    }
    // 时间戳必须严格递增 (DDS best-effort 链路防御)
    if (!buf_.empty() && sample.t.s <= buf_.back().t.s) {
        ++disorder_drop_count_;
        return false;
    }
    buf_.push_back(sample);
    // 剔除超出缓冲时长的旧样本
    const double cutoff = sample.t.s - duration_;
    while (!buf_.empty() && buf_.front().t.s < cutoff) {
        buf_.pop_front();
    }
    return true;
}

OdomBuffer::QueryResult OdomBuffer::query(SampleTime t, pose_math::Pose * out)
{
    if (buf_.empty()) {
        ++too_old_count_;    // EMPTY 健康计数并入 too_old (详设 4.6: 窗口内
        return QueryResult::EMPTY;    // 观测因缓冲为空自然丢弃, 计入 obs_too_old)
    }
    if (t.s < buf_.front().t.s) {
        ++too_old_count_;
        return QueryResult::TOO_OLD;
    }
    if (t.s > buf_.back().t.s) {
        if (t.s - buf_.back().t.s <= max_extrapolation_) {
            *out = buf_.back().pose;    // 等效零阶保持
            return QueryResult::OK;
        }
        ++too_new_count_;
        return QueryResult::TOO_NEW;
    }
    // t ∈ [oldest, newest]: 定位相邻两样本
    auto it = std::lower_bound(
        buf_.begin(), buf_.end(), t.s,
        [](const OdomSample & s, double tv) { return s.t.s < tv; });
    if (it->t.s == t.s || it == buf_.begin()) {
        *out = it->pose;
        return QueryResult::OK;
    }
    const OdomSample & s1 = *it;
    const OdomSample & s0 = *(it - 1);
    const double u = (t.s - s0.t.s) / (s1.t.s - s0.t.s);    // s1.t > s0.t 由 push 保证
    pose_math::Pose interp;
    for (int i = 0; i < 3; ++i) {
        interp.p[i] = s0.pose.p[i] + u * (s1.pose.p[i] - s0.pose.p[i]);
    }
    interp.q = pose_math::quat_slerp(s0.pose.q, s1.pose.q, u);
    *out = interp;
    return QueryResult::OK;
}

void OdomBuffer::clear_and_settle(SampleTime settle_until)
{
    buf_.clear();
    settle_until_ = settle_until;
}

void OdomBuffer::clear()
{
    buf_.clear();
}

}  // namespace map_odom_bias
