// Copyright Epic Games, Inc. All Rights Reserved.


// Auto-generated by spvreflect please don't modify by hand

#pragma once

struct SORReflection
{
	static constexpr uint32_t dsetCount { 1 };


	struct DescSet0_Init
	{
		VkDescriptorSetLayoutBinding bindings[11];
		uint32_t					 nBindings { 11 };

		DescSet0_Init()
		{
			// Binding for:du_curr
			bindings[0] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 0,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:dv_curr
			bindings[1] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 1,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:du_other
			bindings[2] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 2,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:dv_other
			bindings[3] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 3,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:weights_curr
			bindings[4] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 4,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:weights_other
			bindings[5] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 5,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:A11_curr
			bindings[6] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 6,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:A12_curr
			bindings[7] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 7,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:A22_curr
			bindings[8] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 8,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:b1_curr
			bindings[9] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 9,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
			// Binding for:b2_curr
			bindings[10] =
				VkDescriptorSetLayoutBinding
				{
					/*.binding*/ 10,
					/*.descriptorType*/ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
					/*.descriptorCount*/ 1,
					/*.stageFlags*/ VK_SHADER_STAGE_COMPUTE_BIT,
					/*.pImmutableSamplers*/ nullptr
				};
		}
	};

	struct DescSet0_Update
	{
		VkWriteDescriptorSet		writes[11];
		uint32_t					nWrites { 11 };

		VkDescriptorBufferInfo		du_currDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		dv_currDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		du_otherDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		dv_otherDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		weights_currDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		weights_otherDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		A11_currDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		A12_currDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		A22_currDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		b1_currDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };
		VkDescriptorBufferInfo		b2_currDesc{ VK_NULL_HANDLE, 0, VK_WHOLE_SIZE };

		DescSet0_Update(VkBuffer du_curr, VkBuffer dv_curr, VkBuffer du_other, VkBuffer dv_other, VkBuffer weights_curr, VkBuffer weights_other, VkBuffer A11_curr, VkBuffer A12_curr, VkBuffer A22_curr, VkBuffer b1_curr, VkBuffer b2_curr)
		{
			du_currDesc.buffer = du_curr;

			writes[0] = {};
			writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[0].pNext = nullptr;
            writes[0].dstSet = 0;
            writes[0].dstBinding = 0;
            writes[0].descriptorCount = 1;
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[0].pBufferInfo = &du_currDesc;

			dv_currDesc.buffer = dv_curr;

			writes[1] = {};
			writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[1].pNext = nullptr;
            writes[1].dstSet = 0;
            writes[1].dstBinding = 1;
            writes[1].descriptorCount = 1;
            writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[1].pBufferInfo = &dv_currDesc;

			du_otherDesc.buffer = du_other;

			writes[2] = {};
			writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[2].pNext = nullptr;
            writes[2].dstSet = 0;
            writes[2].dstBinding = 2;
            writes[2].descriptorCount = 1;
            writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[2].pBufferInfo = &du_otherDesc;

			dv_otherDesc.buffer = dv_other;

			writes[3] = {};
			writes[3].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[3].pNext = nullptr;
            writes[3].dstSet = 0;
            writes[3].dstBinding = 3;
            writes[3].descriptorCount = 1;
            writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[3].pBufferInfo = &dv_otherDesc;

			weights_currDesc.buffer = weights_curr;

			writes[4] = {};
			writes[4].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[4].pNext = nullptr;
            writes[4].dstSet = 0;
            writes[4].dstBinding = 4;
            writes[4].descriptorCount = 1;
            writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[4].pBufferInfo = &weights_currDesc;

			weights_otherDesc.buffer = weights_other;

			writes[5] = {};
			writes[5].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[5].pNext = nullptr;
            writes[5].dstSet = 0;
            writes[5].dstBinding = 5;
            writes[5].descriptorCount = 1;
            writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[5].pBufferInfo = &weights_otherDesc;

			A11_currDesc.buffer = A11_curr;

			writes[6] = {};
			writes[6].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[6].pNext = nullptr;
            writes[6].dstSet = 0;
            writes[6].dstBinding = 6;
            writes[6].descriptorCount = 1;
            writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[6].pBufferInfo = &A11_currDesc;

			A12_currDesc.buffer = A12_curr;

			writes[7] = {};
			writes[7].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[7].pNext = nullptr;
            writes[7].dstSet = 0;
            writes[7].dstBinding = 7;
            writes[7].descriptorCount = 1;
            writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[7].pBufferInfo = &A12_currDesc;

			A22_currDesc.buffer = A22_curr;

			writes[8] = {};
			writes[8].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[8].pNext = nullptr;
            writes[8].dstSet = 0;
            writes[8].dstBinding = 8;
            writes[8].descriptorCount = 1;
            writes[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[8].pBufferInfo = &A22_currDesc;

			b1_currDesc.buffer = b1_curr;

			writes[9] = {};
			writes[9].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[9].pNext = nullptr;
            writes[9].dstSet = 0;
            writes[9].dstBinding = 9;
            writes[9].descriptorCount = 1;
            writes[9].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[9].pBufferInfo = &b1_currDesc;

			b2_currDesc.buffer = b2_curr;

			writes[10] = {};
			writes[10].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[10].pNext = nullptr;
            writes[10].dstSet = 0;
            writes[10].dstBinding = 10;
            writes[10].descriptorCount = 1;
            writes[10].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[10].pBufferInfo = &b2_currDesc;
		}

	};
};

