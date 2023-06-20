// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanCopyBuffer.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_CopyBuffer.comp.spv.h"

#include "shaders/CopyBuffer.comp.h"
#include "shaders/CopyBuffer.comp.refl.h"

#include <cmath>

namespace epic::nls
{
	bool VulkanCopyBuffer::Init()
	{
		return Kernel::Init<CopyBufferReflection, CopyBufferPushConstants>(SPV_CopyBuffer_comp_spv, sizeof(SPV_CopyBuffer_comp_spv));
	}

	void VulkanCopyBuffer::Copy(const std::shared_ptr<VulkanComputeBuffer>& input,
								const std::shared_ptr<VulkanComputeBuffer>& output,
								VulkanComputeContext::HBatch batch /* = nullptr */)
	{
		CARBON_PRECONDITION(input, "input buffer must be valid");
		CARBON_PRECONDITION(output, "output buffer must be valid");
		CARBON_PRECONDITION(input->Width() <= output->Width() &&
		                    input->Height() <= output->Height(), "output buffer must have enough space to hold the input buffer");

		const int width = input->Width();
		const int height = input->Height();

		CopyBufferPushConstants ubo;
		ubo.inputWidth = width;
		ubo.inputHeight = height;
		ubo.outputWidth = output->Width();

		CopyBufferReflection::DescSet0_Update dsu{
			input->ManagedBuffer()->Buffer(),
			output->ManagedBuffer()->Buffer()
		};

		const uint32_t numGroupsX = static_cast<uint32_t>(std::ceil(width / static_cast<float>(CopyBufferThreadSizeX)));
		const uint32_t numGroupsY = static_cast<uint32_t>(std::ceil(height / static_cast<float>(CopyBufferThreadSizeY)));

		Run(batch,
			{ numGroupsX, numGroupsY, 1 },
			dsu.writes,
			dsu.nWrites,
			&ubo);
	}
}
