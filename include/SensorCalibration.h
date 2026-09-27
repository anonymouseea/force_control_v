#pragma once

#include "ControlConfig.h"
#include "ControlTypes.h"
#include <cmath>

// 按时间戳分步累计原始数据，采样完成前不覆盖原有零点。
class SensorCalibration {
public:
    void begin(ControlTime now) {
        active_ = true;
        count_ = 0;
        large_sum_.fill(0.0);
        small_sum_.fill(0.0);
        next_sample_ = now + std::chrono::milliseconds(ControlConfig::ZERO_WARMUP_MS);
    }
    void cancel() { active_ = false; }
    bool active() const { return active_; }
    bool sampleDue(ControlTime now) const { return active_ && now >= next_sample_; }
    ZeroResult sample(ControlTime now, const ForceSample& large, const ForceSample& small) {
        if (!active_) return ZeroResult::Failed;
        if (!sampleDue(now)) return ZeroResult::Pending;
        for (int i = 0; i < 6; ++i) {
            if (!std::isfinite(large[i]) || !std::isfinite(small[i])) {
                active_ = false;
                return ZeroResult::Failed;
            }
        }
        for (int i = 0; i < 6; ++i) {
            large_sum_[i] += large[i];
            small_sum_[i] += small[i];
        }
        ++count_;
        // 超期只采当前一帧，不补采重复数据。
        next_sample_ = now + std::chrono::milliseconds(ControlConfig::ZERO_SAMPLE_MS);
        if (count_ < ControlConfig::ZERO_SAMPLE_COUNT) return ZeroResult::Pending;
        for (int i = 0; i < 6; ++i) {
            large_offset_[i] = large_sum_[i] / count_;
            small_offset_[i] = small_sum_[i] / count_;
        }
        active_ = false;
        return ZeroResult::Complete;
    }
    const ForceSample& offset(bool small) const { return small ? small_offset_ : large_offset_; }

private:
    bool active_ = false;
    int count_ = 0;
    ControlTime next_sample_{};
    ForceSample large_sum_ = {{0, 0, 0, 0, 0, 0}};
    ForceSample small_sum_ = {{0, 0, 0, 0, 0, 0}};
    ForceSample large_offset_ = {{0, 0, 0, 0, 0, 0}};
    ForceSample small_offset_ = {{0, 0, 0, 0, 0, 0}};
};
