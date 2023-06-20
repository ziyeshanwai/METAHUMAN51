// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/utils/TaskThreadPool.h>

namespace epic::carbon {

std::shared_ptr<TaskThreadPool> TaskThreadPool::GlobalInstance(bool createIfNotAvailable)
{
    static std::mutex mutex;
    static std::weak_ptr<TaskThreadPool> weakPtr;
    std::unique_lock<std::mutex> lock(mutex);
    auto ptr = weakPtr.lock();
    if (!ptr && createIfNotAvailable) {
        ptr = std::make_shared<TaskThreadPool>(std::max<int>(1, std::thread::hardware_concurrency() / 2));
        weakPtr = ptr;
    }
    return ptr;
}


TaskThreadPool::TaskThreadPool(int numThreads) : m_stop(false) {
    LOG_INFO("Creating thread pool with {} threads.", numThreads);
    for (int i = 0; i < numThreads; ++i) {
        m_workerThreads.emplace_back(std::thread([&](int threadNum) {
#if defined(__APPLE__)
            std::string threadName = std::string("TaskThread ") + std::to_string(threadNum);
            pthread_setname_np(threadName.c_str());
#elif !defined(_MSC_VER)
            std::string threadName = std::string("TaskThread ") + std::to_string(threadNum);
            pthread_setname_np(pthread_self(), threadName.c_str());
#else
            (void)threadNum;
#endif
            RunWorkerThread();
        }, i));
    }
}

TaskThreadPool::~TaskThreadPool() {
    Stop();
}

void TaskThreadPool::Stop()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_stop || m_workerThreads.empty()) return;

    m_stop = true;
    lock.unlock();
    m_conditionVariable.notify_all();
    for (size_t i = 0; i < m_workerThreads.size(); ++i) {
        if (m_workerThreads[i].joinable()) {
            // for now detach here as we have a strange bug on windows where this will not join even if the threads have finished
            m_workerThreads[i].join();
        } else {
            LOG_ERROR("thread {} is not joinable", i);
        }
    }
    m_workerThreads.clear();
}

void TaskThreadPool::RunWorkerThread() {
    while (true) {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (m_tasks.size() > 0) {
            std::packaged_task<void()> packagedTask = std::move(m_tasks.front());
            m_tasks.pop_front();
            lock.unlock();
            if (packagedTask.valid()) {
                packagedTask();
            }
        } else if (m_stop) {
            return;
        } else {
            m_conditionVariable.wait(lock);
        }
    }
}

void TaskThreadPool::RunTask()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_tasks.size() > 0) {
        std::packaged_task<void()> packagedTask = std::move(m_tasks.front());
        m_tasks.pop_front();
        lock.unlock();
        if (packagedTask.valid()) {
            packagedTask();
        }
    }
}

TaskFuture TaskThreadPool::AddTask(std::function<void()>&& task)
{
    if (m_stop) {
        CARBON_CRITICAL("no tasks should be added when the thread pool has been stopped");
    }
    if (!task) {
        return TaskFuture(std::future<void>(), this);
    }
    std::unique_lock<std::mutex> lock(m_mutex);
    std::packaged_task<void()> packagedTask(std::move(task));
    std::future<void> future = packagedTask.get_future();
    m_tasks.emplace_back(std::move(packagedTask));
    lock.unlock();
    m_conditionVariable.notify_one();
    return TaskFuture(std::move(future), this);
}

void TaskThreadPool::AddTaskRangeAndWait(int numTasks, const std::function<void(int, int)>& processFunction)
{
    if (m_stop) {
        CARBON_CRITICAL("no tasks should be added when the thread pool has been stopped");
    }
    if (numTasks == 0) return;

    std::unique_lock<std::mutex> lock(m_mutex);

    const int numThreadsToUse = std::min<int>(numTasks, static_cast<int>(NumThreads()));
    const int tasksPerThread = numTasks / numThreadsToUse;
    const int additionalTasks = numTasks - tasksPerThread * numThreadsToUse;
    std::vector<std::future<void>> futures;
    futures.reserve(numThreadsToUse);
    int taskIndex = 0;

    for (int k = 0; k < numThreadsToUse; ++k) {
        const int numTasksForThisThread = (k < additionalTasks) ? (tasksPerThread + 1) : tasksPerThread;

        std::packaged_task<void()> packagedTask(std::bind([](const std::function<void(int, int)>& processFunction, int start, int size){
            processFunction(start, start + size);
        }, processFunction, taskIndex, numTasksForThisThread));
        futures.emplace_back(packagedTask.get_future());
        m_tasks.emplace_back(std::move(packagedTask));
        taskIndex += numTasksForThisThread;
    }
    lock.unlock();
    m_conditionVariable.notify_all();

    for (auto& future : futures) {
        while (future.wait_for(std::chrono::seconds(0)) == std::future_status::timeout) {
            // run other tasks while waiting
            RunTask();
        }
    }
    if (taskIndex != numTasks) {
        CARBON_CRITICAL("incorrect parallel call");
    }
}

bool TaskFuture::Valid() const
{
    return m_future.valid();
}

bool TaskFuture::Ready() const
{
    return (m_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready);
}

void TaskFuture::Wait()
{
    while (!Ready()) {
        m_pool->RunTask();
    }
}

TaskFutures::~TaskFutures()
{
    Wait();
}

void TaskFutures::Add(TaskFuture&& future)
{
    m_futures.emplace_back(std::move(future));
}

void TaskFutures::Wait()
{
    for (size_t i = 0; i < m_futures.size(); ++i) {
        m_futures[i].Wait();
    }
    m_futures.clear();
}

} //namespace epic::carbon
