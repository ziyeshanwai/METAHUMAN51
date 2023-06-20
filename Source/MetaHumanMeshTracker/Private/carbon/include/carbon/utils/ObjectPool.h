// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>

#include <memory>
#include <mutex>
#include <stack>

namespace epic::carbon {

/**
 * Class to keep a pool of objects. Aquire() returns a shared_ptr to an object of the pool, or it creates a new one if the pool is empty.
 * Upon release of the shared_ptr, the object is returned to the pool via Release(). The class
 * is intended to be used to keep large allocated data structures in the pool and therefore prevent constant mallocs and frees.
 * @warning neither constructor or destructor are called on the objects upon Aquire() and Release(). The caller
 * is responsible to fill all members of the returned object.
 */
template <typename T>
class ObjectPool
{
public:
    ObjectPool() : m_selfPtr(std::make_shared<ObjectPool<T>*>(this)) {}

    std::shared_ptr<T> Aquire()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        T* rawPtr;
        if (m_objects.size() > 0) {
            rawPtr = m_objects.top().release();
            m_objects.pop();
        } else {
            rawPtr = new T;
            m_totalCreatedObjects++;
        }
        std::weak_ptr<ObjectPool<T>*> poolWeakPtr = m_selfPtr;
        auto releaseFunctor = [poolWeakPtr](T* ptr) {
            if (auto poolPtr = poolWeakPtr.lock()) {
                (*poolPtr)->Release(std::unique_ptr<T>(ptr));
            }
            else {
                std::default_delete<T>()(ptr);
            }
        };
        return std::shared_ptr<T>(rawPtr, releaseFunctor);
    }

    size_t Size() const
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_objects.size();
    }

    //! Clears all objects from the pool that are currently within the pool. Does not remove the ones that have been aquired.
    void Clear()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        while (m_objects.size() > 0) {
            m_objects.pop();
            m_totalCreatedObjects--;
        }
    }

    //! @returns the total number of objects belonging to the pool, both currently in the pool and the ones that have been aquired.
    size_t TotalSize() const
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_totalCreatedObjects;
    }

private:
    void Release(std::unique_ptr<T>&& obj)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_objects.push(std::move(obj));
    }

private:
    mutable std::mutex m_mutex;
    std::stack<std::unique_ptr<T>> m_objects;
    std::shared_ptr<ObjectPool<T>*> m_selfPtr;
    size_t m_totalCreatedObjects = 0;
};

} // namespace epic::carbon
