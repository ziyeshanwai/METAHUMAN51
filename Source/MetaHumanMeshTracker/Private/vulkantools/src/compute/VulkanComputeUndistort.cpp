// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanComputeUndistort.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include "shaders/embed_UndistortImage.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/UndistortImage.comp.h"
#include "shaders/UndistortImage.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanComputeUndistort::Init()
{
    return Kernel::Init< UndistortImageReflection, UndistortImagePushConstants >(SPV_UndistortImage_comp_spv, sizeof(SPV_UndistortImage_comp_spv));
}

void VulkanComputeUndistort::Undistort(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    const MetaShapeCamera<float>& camera,
    const Camera<float>& rectifiedCamera,
    InterpolationMethod interpolationMethod,
    VulkanComputeContext::HBatch batch
)
{
    UndistortImagePushConstants ubo = {};

    // homography from new camera to old camera
    Eigen::Map<Eigen::Matrix3f>(ubo.H, 3, 3) = camera.Intrinsics() * camera.Extrinsics().Linear() * rectifiedCamera.Extrinsics().Linear().transpose() * rectifiedCamera.Intrinsics().inverse();
    Eigen::Map<Eigen::Vector4f>(ubo.K, 4, 1) = camera.RadialDistortion();
    Eigen::Map<Eigen::Vector4f>(ubo.P, 4, 1) = camera.TangentialDistortion();
    Eigen::Map<Eigen::Vector2f>(ubo.B, 2, 1) = camera.Skew();
    ubo.fx = camera.Intrinsics()(0,0);
    ubo.fy = camera.Intrinsics()(1,1);
    ubo.cx = camera.Intrinsics()(0,2);
    ubo.cy = camera.Intrinsics()(1,2);
    ubo.interpolationMethod = int(interpolationMethod);
    ubo.width = rectifiedCamera.Width();
    ubo.height = rectifiedCamera.Height();
    ubo.maxx = camera.Width() - 1;
    ubo.maxy = camera.Height() - 1;

    // WriteDescriptorSet for input and output buffer
    UndistortImageReflection::DescSet0_Update dsu(inputBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {    
            (uint32_t)std::ceil(rectifiedCamera.Width() / float(UndistortImageComputeThreadSizeX)),
            (uint32_t)std::ceil(rectifiedCamera.Height() / float(UndistortImageComputeThreadSizeY)),
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
