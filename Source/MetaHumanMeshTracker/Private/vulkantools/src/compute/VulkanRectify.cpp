// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanRectify.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include "shaders/embed_Rectify.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/Rectify.comp.h"
#include "shaders/Rectify.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanRectify::Init()
{
    return Kernel::Init< RectifyReflection, RectifyPushConstants >(SPV_Rectify_comp_spv, sizeof(SPV_Rectify_comp_spv));
}

void VulkanRectify::Rectify(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    const Camera<float>& camera,
    const Camera<float>& rectifiedCamera,
    InterpolationMethod interpolationMethod,
    VulkanComputeContext::HBatch batch
)
{
    RectifyPushConstants ubo = {};

    // homography from new camera to old camera
    Eigen::Map<Eigen::Matrix3f>(ubo.H, 3, 3) = camera.Intrinsics() * camera.Extrinsics().Linear() * rectifiedCamera.Extrinsics().Linear().transpose() * rectifiedCamera.Intrinsics().inverse();
    ubo.interpolationMethod = int(interpolationMethod);
    ubo.width = rectifiedCamera.Width();
    ubo.height = rectifiedCamera.Height();
    ubo.maxx = camera.Width() - 1;
    ubo.maxy = camera.Height() - 1;

    RectifyReflection::DescSet0_Update   dsu(inputBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(
        batch,
        {
            (uint32_t)std::ceil(rectifiedCamera.Width() / float(RectifyComputeThreadSizeX)),
            (uint32_t)std::ceil(rectifiedCamera.Height() / float(RectifyComputeThreadSizeY)),
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo
    );
}

} // namespace epic::nls
