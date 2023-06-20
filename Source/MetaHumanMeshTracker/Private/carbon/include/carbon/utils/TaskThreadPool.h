// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <future>
#include <thread>
#include <vector>

namespace epic::carbon {

class TaskFuture;
class TaskThreadPool
{
public:
    static std::shared_ptr<TaskThreadPool> GlobalInstance(bool createIfNotAvailable);

    TaskThreadPool(int numThreads);
    ~TaskThreadPool();

    void Stop();

    size_t NumThreads() const { return m_workerThreads.size(); }

	TaskFuture AddTask(std::function<void()>&& task);

    void AddTaskRangeAndWait(int numTasks, const std::function<void(int, int)>& processFunction);

    //! explicitly run a task
    void RunTask();

private:

    void RunWorkerThread();

private:
    std::atomic<bool> m_stop;
    std::mutex m_mutex;
    std::condition_variable m_conditionVariable;
    std::deque<std::packaged_task<void()>> m_tasks;
    std::vector<std::thread> m_workerThreads;
};

class TaskFuture {
public:
    TaskFuture() : m_future(), m_pool(nullptr) {}
    TaskFuture(std::future<void>&& future, TaskThreadPool* pool) : m_future(std::move(future)), m_pool(pool) {}
    TaskFuture(TaskFuture&&) = default;
    TaskFuture(const TaskFuture&) = delete;
    TaskFuture& operator=(TaskFuture&&) = default;
    TaskFuture& operator=(const TaskFuture&) = delete;

    bool Valid() const;
    bool Ready() const;
    void Wait();

private:
    std::future<void> m_future;
    TaskThreadPool* m_pool;
};

class TaskFutures {
public:
    TaskFutures() = default;
    ~TaskFutures();

    void Reserve(size_t size) { m_futures.reserve(size); }

    void Add(TaskFuture&& future);
    void Wait();

private:
    std::vector<TaskFuture> m_futures;
};

} // namespace epic::carbon
