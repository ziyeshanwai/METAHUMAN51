// Copyright Epic Games, Inc. All Rights Reserved.


// Auto-generated by spvreflect please don't modify by hand

#pragma once

struct ExtractChannelUint8Reflection
{
	static constexpr uint32_t dsetCount { 1 };


	struct DescSet0_Init
	{
		VkDescriptorSetLayoutBinding bindings[2];
		uint32_t					 nBindings { 2 };

		DescSet0_Init()
		{
			// Binding for:inputImage
			bindings[0] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 0,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:outputImage
			bindings[1] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 1,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
		}
	};

	struct DescSet0_Update
	{
		VkWriteDescriptorSet		writes[2];
		uint32_t					nWrites { 2 };

		VkDescriptorBufferInfo		inputImageDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		outputImageDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };

		DescSet0_Update(VkBuffer inputImage, VkBuffer outputImage)
		{
			inputImageDesc.buffer = inputImage;

			writes[0] = {};
			writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[0].pNext = nullptr;
            writes[0].dstSet = 0;
            writes[0].dstBinding = 0;
            writes[0].descriptorCount = 1;
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[0].pBufferInfo = &inputImageDesc;

			outputImageDesc.buffer = outputImage;

			writes[1] = {};
			writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[1].pNext = nullptr;
            writes[1].dstSet = 0;
            writes[1].dstBinding = 1;
            writes[1].descriptorCount = 1;
            writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[1].pBufferInfo = &outputImageDesc;
		}

	};
};

