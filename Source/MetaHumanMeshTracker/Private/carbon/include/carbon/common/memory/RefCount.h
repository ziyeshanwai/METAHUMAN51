// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <atomic>
#include <cstdint>
#include <carbon/common/Common.h>

namespace epic {
namespace carbon {
namespace memory {

/*
@Brief RefCount represents basic implementation class for reference counting.
It relies on std::atomic field.
<br> RefCount class object should be stored as ptr member of class which is intended for reference counting.
*/
class EPIC_CARBON_API RefCount final {
    public:
        /*
        Constructor.
        */
        RefCount() noexcept;

        /*
        Destructor.
        */
        ~RefCount() noexcept;

        /*
        Increments reference count.
        */
        void AddRef() noexcept;

        /*
        Decrements reference count.

        @return current reference count after decrement.
        */
        std::int32_t Release() noexcept;

    private:
        std::atomic<std::int32_t> m_refCount{};
};

}  // namespace memory
}  // namespace carbon
}  // namespace epic
