// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>

#include <functional>
#include <memory>
#include <mutex>

namespace epic::carbon {
    //! @returns True if the two pointers have the same control block i.e. they point to the same object (though .get() may point to different parts of the same object, see https://en.cppreference.com/w/cpp/memory/shared_ptr/owner_before).
    template<typename T, typename U>
    inline bool IsPointerControlBlockEqual(const std::weak_ptr<T>& t, const std::shared_ptr<U>& u) {
        return !t.owner_before(u) && !u.owner_before(t);
    }

    template<typename T, typename U>
    inline bool IsPointerControlBlockEqual(const std::shared_ptr<T>& t, const std::weak_ptr<U>& u) {
        return !t.owner_before(u) && !u.owner_before(t);
    }

    template<typename T, typename U>
    inline bool IsPointerControlBlockEqual(const std::shared_ptr<T>& t, const std::shared_ptr<U>& u) {
        return !t.owner_before(u) && !u.owner_before(t);
    }

    template<typename T, typename U>
    inline bool IsPointerControlBlockEqual(const std::weak_ptr<T>& t, const std::weak_ptr<U>& u) {
        return !t.owner_before(u) && !u.owner_before(t);
    }

    /**
     * Utility class that can be used to generate data based on some reference input data, but cache the output as long
     * as the input pointer does not change. @see Update()
     */
    template<typename OUT>
    class Cache {
        public:
            Cache() = default;
            ~Cache() = default;

            Cache(const Cache& other) {
                std::lock_guard<std::mutex> lock(other.mutex);
                prevRefData = other.prevRefData;
                result = other.result;
            }

            Cache& operator=(const Cache& other) {
                if (this != &other) {
                    Cache tmp(other);
                    std::lock_guard<std::mutex> lock(mutex);
                    std::swap(prevRefData, tmp.prevRefData);
                    std::swap(result, tmp.result);
                }
                return *this;
            }

            /**
             * Updates the cache by processing the reference input data. If the reference input data is the
             * same (i.e. the pointers point to the same object), then the cached result is returned, otherwise
             * a new result is calculated by calling @p createFunc() and that result is cached and returned.
             * The Cache works by assuming that as long as @p refData pointer does not change, the createFunc() will return the exact same result.
             * It is the caller's responsibility to ensure this.
             */
            std::shared_ptr<const OUT> Update(std::shared_ptr<const void> refData,
                                              std::function<std::shared_ptr<const OUT>()> createFunc) {
                std::lock_guard lock(mutex);
                if (!IsPointerControlBlockEqual(prevRefData, refData)) {
                    prevRefData = refData;
                    if (refData) {
                        result = createFunc();
                    } else {
                        result.reset();
                    }
                }
                return result;
            }

            void Reset() {
                std::lock_guard lock(mutex);
                prevRefData.reset();
                result.reset();
            }

        private:
            Cache(const Cache&& other) = delete;
            Cache& operator=(const Cache&&) = delete;

        private:
            mutable std::mutex mutex;
            std::weak_ptr<const void> prevRefData;
            std::shared_ptr<const OUT> result;
    };
}  // namespace epic::nls
