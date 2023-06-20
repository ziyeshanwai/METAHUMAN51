// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>

#include <list>
#include <mutex>
#include <optional>

namespace epic::carbon {

/**
 * @brief Simple most recently used queue storing a list of keys and values. Iterates through the queue linearly, so it is most optimal if the queue is requesting the last used object.
 *
 * @tparam Key  The key that is used to index the object.
 * @tparam T    The object/value.
 */
template <typename Key, typename T>
class MRUQueue
{
public:
    MRUQueue(int queueSize)
        : m_queueSize(std::max<int>(0, queueSize))
        {}

    //! @returns True if item with @p key is in the MRU queue. Note that there is no guarantee that FetchObject then returns the object, as it might disappear in the meantime.
    bool HasObject(const Key& key) const
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        for (auto it = m_items.begin(); it != m_items.end(); ++it) {
            if (it->first == key) {
                return true;
            }
        }
        return false;
    }

    //! @returns The value for @p key in the MRU queue. Puts the object to the front of the most recently used queue.
    std::optional<T> FetchObject(const Key& key)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        for (auto it = m_items.begin(); it != m_items.end(); ++it) {
            if (it->first == key) {
                const T value = it->second;
                m_items.erase(it);
                m_items.push_front({key, value});
                return value;
            }
        }
        return std::optional<T>();
    }

    //! Adds the key/object pair. Note that it does not check if the key is already in the queue. This is the callers responsibility.
    void AddObject(const Key& key, const T& object)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_items.push_front({key, object});
        if (m_items.size() > m_queueSize) {
            m_items.pop_back();
        }
    }

    //! @returns the size of the most recently used queue
    size_t QueueSize() const { return m_queueSize; }

    //! @returns the current number of entries in the most recently used queue (less or equal to QueueSize())
    size_t NumEntries() const
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_items.size();
    }

    //! Clears the most recently used queue
    void Clear()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_items.clear();
    }

private:
    mutable std::mutex m_mutex;
    std::list<std::pair<Key, T>> m_items;
    size_t m_queueSize = 0;
};

} // namespace epic::carbon
