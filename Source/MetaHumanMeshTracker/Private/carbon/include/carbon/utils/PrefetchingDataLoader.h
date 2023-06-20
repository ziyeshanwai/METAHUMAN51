// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/utils/MRUQueue.h>
#include <carbon/utils/TaskThreadPool.h>

#include <functional>
#include <list>
#include <memory>
#include <mutex>

namespace epic::carbon {

template <typename T>
class PrefetchingDataLoader {

public:
    PrefetchingDataLoader(int cacheSize, std::function<T(int)> funcGenerateData, std::function<bool(int)> funcIsValidFrame)
        : m_mutex()
        , m_taskThreadPool(TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/true))
        , m_recentData(cacheSize)
        , m_prefetchTasks()
        , m_funcGenerateData(funcGenerateData)
        , m_funcIsValidFrame(funcIsValidFrame)
    {
    }

    ~PrefetchingDataLoader()
    {
        ClearPrefetchList(/*waitForAll=*/true);
    }

    //! Retrieves frame @p frameNumber. Note that this should not be called from a task thread.
    T GetData(int frameNumber)
    {
        // validate that the requested frame is valid
        if (!m_funcIsValidFrame(frameNumber)) {
            CARBON_CRITICAL("invalid frame requested from prefetching data loader");
        }

        std::unique_lock<std::mutex> lock(m_mutex);

        // remove all finished tasks from the prefetch list
        ClearPrefetchList(/*waitForAll-*/false);

        // check if the frame is in the recent image queue
        std::optional<T> cachedFrame = m_recentData.FetchObject(frameNumber);
        if (cachedFrame.has_value()) {
            PrefetchAround(frameNumber);
            return cachedFrame.value();
        }

        // check if the frame is currently being prefetched
        if (m_prefetchTasks.size() > 0) {
            for (auto it = m_prefetchTasks.begin(); it != m_prefetchTasks.end(); ++it) {
                const bool isFrame = (it->first == frameNumber);
                if (isFrame) {
                    // wait for prefetch to get the image
                    it->second.Wait();
                    // the frame has been prefetched, hence we need to get it again from the recent image queue
                    cachedFrame = m_recentData.FetchObject(frameNumber);
                    if (cachedFrame.has_value()) {
                        PrefetchAround(frameNumber);
                        return cachedFrame.value();
                    } else {
                        LOG_ERROR("logic error: data is not in queue after prefetch");
                    }
                }
            }
        }

        // Wait for all prefetch tasks before reading the frame below.
        // This ensures that m_funcGenerateData() is not being called from the prefetch thread and the calling thread.
        ClearPrefetchList(/*waitForAll=*/true);

        // generate the data fro the frame, add it to the recent data queue and prefetch data around the frame
        auto result = m_funcGenerateData(frameNumber);
        m_recentData.AddObject(frameNumber, result);
        PrefetchAround(frameNumber);
        return result;
    }

    //! Clears all finished prefetch tasks from the list
    void ClearPrefetchList(bool waitForAll)
    {
        if (waitForAll) {
            for (auto it = m_prefetchTasks.begin(); it != m_prefetchTasks.end(); ++it) {
                it->second.Wait();
            }
            m_prefetchTasks.clear();
        } else {
            for (auto it = m_prefetchTasks.begin(); it != m_prefetchTasks.end(); /*iterator is increased in body*/) {
                if (it->second.Ready()) {
                    it = m_prefetchTasks.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

private:
    //! Prefetches the next frame after @p frameNumber, but if that already exists, then it prefetches the previous frame instead.
    void PrefetchAround(int frameNumber)
    {
        if (!m_recentData.HasObject(frameNumber + 1)) {
            Prefetch(frameNumber + 1);
        } else if (!m_recentData.HasObject(frameNumber - 1)) {
            Prefetch(frameNumber - 1);
        }
    }

    //! Pretch frame @p frameNumber if it is valid and not yet in the queue
    void Prefetch(int frameNumber)
    {
        if (!m_funcIsValidFrame(frameNumber)) return;
        if (m_recentData.HasObject(frameNumber)) return;

        m_prefetchTasks.push_back({frameNumber, m_taskThreadPool->AddTask(std::bind([&](int frame) {
            T result = m_funcGenerateData(frame);
            m_recentData.AddObject(frame, result);
        }, frameNumber))});
    }

private:
    std::mutex m_mutex;
    //! thread pool that is used to prefetch data
    std::shared_ptr<TaskThreadPool> m_taskThreadPool;
    //! most recently used data
    MRUQueue<int, T> m_recentData;
    //! a list of prefetch tasks
    std::list<std::pair<int, TaskFuture>> m_prefetchTasks;
    //! client-provided function that generates data for a frame index
    std::function<T(int)> m_funcGenerateData;
    //! client-provided function that returns whether a frame index is valid
    std::function<bool(int)> m_funcIsValidFrame;
};

} // namespace epic::carbon
