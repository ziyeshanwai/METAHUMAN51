// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanFillBuffer.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_FillBuffer.comp.spv.h"

#include "shaders/FillBuffer.comp.h"
#include "shaders/FillBuffer.comp.refl.h"

#include <cmath>

namespace epic::nls
{
	bool VulkanFillBuffer::Init()
	{
		return Kernel::Init<FillBufferReflection, FillBufferPushConstants>(SPV_FillBuffer_comp_spv, sizeof(SPV_FillBuffer_comp_spv));
	}

	void VulkanFillBuffer::Fill(const std::shared_ptr<VulkanComputeBuffer>& buffer,
								float value /* = 0.0f */,
								VulkanComputeContext::HBatch batch /* = nullptr */)
	{
		CARBON_PRECONDITION(buffer, "buffer must be valid");

		const int width = buffer->Width();
		const int height = buffer->Height();

		FillBufferPushConstants ubo;
		ubo.width = width;
		ubo.height = height;
		ubo.value = value;

		FillBufferReflection::DescSet0_Update dsu{
			buffer->ManagedBuffer()->Buffer()
		};

		const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(width / static_cast<float>(FillBufferThreadSizeX)));
		const uint32_t numGroupsY = static_cast<uint32_t>(std::ceil(height / static_cast<float>(FillBufferThreadSizeY)));

		Run(batch,
			{ numGroupsX, numGroupsY, 1 },
			dsu.writes,
			dsu.nWrites,
			&ubo);
	}
}
