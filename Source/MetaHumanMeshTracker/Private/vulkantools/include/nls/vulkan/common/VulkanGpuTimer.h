// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "VulkanDevice.h"

namespace epic {
namespace nls {

//! Helper class for querying GPU timestamps
class VulkanGpuTimer
{
public:

	//! Create timer with max. number of timestamps
	static std::unique_ptr< VulkanGpuTimer > Create(std::shared_ptr< VulkanDevice > dev, uint32_t nTimestamps = 128)
	{
		auto gpuTimer = new VulkanGpuTimer();

		if (!gpuTimer->Init(dev, nTimestamps))
		{
			//LOG_ERROR("Failed to create VulkanGPUTimer");
			delete gpuTimer;
			gpuTimer = nullptr;
		}

		return std::unique_ptr< VulkanGpuTimer >(gpuTimer);
	}

	//! Cleanup device resources
	~VulkanGpuTimer()
	{
		if (m_queryPool)
		{
			vkDestroyQueryPool(m_dev->Device(), m_queryPool, nullptr);
			m_queryPool = nullptr;
		}
	}

	//! Reset the queries
	void Reset()
	{
		vkResetQueryPoolEXT(m_dev->Device(), m_queryPool, 0, m_size);
		m_curr = 0;
	}

	//! Record timestamp for pipeline stage
	int32_t AddTimestamp(VkCommandBuffer cmdBuff, VkPipelineStageFlagBits pipeStage)
	{
		if (m_curr < m_size)
		{
			vkCmdWriteTimestamp(cmdBuff, pipeStage, m_queryPool, m_curr);
			++m_curr;

			return int32_t(m_curr) - 1;
		}

		return -1;
	}

	//! Query timestamp values in range [first, first + count - 1]
	//! Timestamp values are recorded from previous Reset()
	//! NOTE: This is a blocking call
	bool QueryValues(uint64_t* timestamps, uint32_t count, uint32_t first = 0)
	{
		VkResult res;

		if (m_curr)
		{
			res = vkGetQueryPoolResults(
					m_dev->Device(),
					m_queryPool,
					first, count,
					count * sizeof(uint64_t),
					timestamps,
					sizeof(uint64_t),
					VK_QUERY_RESULT_WAIT_BIT | VK_QUERY_RESULT_64_BIT);
		}
		else
			res = VK_INCOMPLETE;

		if (res == VK_SUCCESS)
		{
			for (uint32_t c = 0; c < count; ++c)
				timestamps[c] *= m_timestampPeriod;
		}

		return res == VK_SUCCESS;
	}

private:

	//! Default constructor
	VulkanGpuTimer() = default;

	//! Initialize query pool (called by Create())
	bool Init(std::shared_ptr< VulkanDevice > dev, uint32_t nTimestamps)
	{
		m_dev = dev;
		m_timestampPeriod = (uint64_t) dev->Properties().limits.timestampPeriod;

		VkQueryPoolCreateInfo createInfo = {};
		createInfo.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;

		createInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
		createInfo.queryCount = nTimestamps;

		VkResult res = vkCreateQueryPool(m_dev->Device(), &createInfo, nullptr, &m_queryPool);

		m_size = res == VK_SUCCESS ? nTimestamps : 0;
		m_curr = 0;

		if (m_size)
			vkResetQueryPoolEXT(m_dev->Device(), m_queryPool, 0, m_size);

		return res == VK_SUCCESS;
	}

	using TimestampPtr = std::unique_ptr< uint64_t[] >;
	using DevicePtr = std::shared_ptr< VulkanDevice >;

	VkQueryPool			m_queryPool{ nullptr };
	uint32_t			m_size{ 0 };
	uint32_t			m_curr{ 0 };
	uint64_t			m_timestampPeriod{ 0 };
	DevicePtr			m_dev;
};

} // nls
} // epic
