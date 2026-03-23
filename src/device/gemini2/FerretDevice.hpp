// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

// CR-Scan Ferret device handler
// The Ferret is a Creality-branded Orbbec Gemini 215 (MX6600 ASIC).
// It uses the G2/Gemini2 protocol but the private filter library is not
// activated for its PID (0x069C), so frame processors are not attached
// to sensors. The device otherwise works identically to the Gemini 215.

#pragma once
#include "DeviceBase.hpp"
#include "IDeviceManager.hpp"
#include "IFrameTimestamp.hpp"
#include "frameprocessor/FrameProcessor.hpp"

#include <map>
#include <memory>

namespace libobsensor {

class FerretDevice : public DeviceBase {
public:
    FerretDevice(const std::shared_ptr<const IDeviceEnumInfo> &info);
    virtual ~FerretDevice() noexcept override;

    std::vector<std::shared_ptr<IFilter>> createRecommendedPostProcessingFilters(OBSensorType type) override;

private:
    void init() override;
    void initSensorList();
    void initProperties();
    void initSensorStreamProfile(std::shared_ptr<ISensor> sensor);

private:
    const uint64_t                                              frameTimeFreq_  = 1000000;
    uint64_t                                                    deviceTimeFreq_ = 1000000;
    std::function<std::shared_ptr<IFrameTimestampCalculator>()> videoFrameTimestampCalculatorCreator_;
};

}  // namespace libobsensor
