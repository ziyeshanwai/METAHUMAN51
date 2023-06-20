// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#if defined(__APPLE__) && __APPLE__
    #include <experimental/memory_resource>
#else
    #include <memory_resource>
#endif
#include <carbon/common/Common.h>

namespace epic {
namespace carbon {
namespace memory {

#if defined(__APPLE__) && __APPLE__
    using MemoryResource = std::experimental::pmr::memory_resource;
#else
    using MemoryResource = std::pmr::memory_resource;
#endif

/*
@Brief DefaultMemoryResource class represents std::pmr::memory_resource implementation
<br>intended to be used by std::pmr::polymorphic_allocator to create objects.
*/
class EPIC_CARBON_API DefaultMemoryResource : public MemoryResource {
    public:
        /*
        Constructor.
        */
        DefaultMemoryResource() noexcept;

        /*
        Destructor.
        */
        ~DefaultMemoryResource() noexcept;

    private:
        void* do_allocate(size_t size, size_t align) override;
        void do_deallocate(void* ptr, size_t bytes, size_t align) override;
        bool do_is_equal(const MemoryResource& other) const noexcept override;
};
}  // namespace memory
}  // namespace carbon
}  // namespace epic
