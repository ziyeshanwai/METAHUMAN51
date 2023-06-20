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

/*
@Brief context data associated with MemoryResource
*/
struct EPIC_CARBON_API memory_context {
    /*
    Alignment value.
    */
    std::size_t alignment{64L};
};

#if defined(__APPLE__) && __APPLE__
    using MemoryResource = std::experimental::pmr::memory_resource;
#else
    using MemoryResource = std::pmr::memory_resource;
#endif

/*
@Brief AlignedMemoryResource class represents std::pmr::memory_resource impplementation
<br>intended to be used by std::pmr::polymorphic_allocator to create aligned objects.
*/
class EPIC_CARBON_API AlignedMemoryResource : public MemoryResource {
    public:
        /*
        Constructor.

        @param[in] context Additional contect provided to resource such as alignment.
        */
        AlignedMemoryResource(const memory_context& context = {64L}) noexcept;

        /*
        Destructor.
        */
        ~AlignedMemoryResource() noexcept;

    private:
        memory_context m_context{};
        void* do_allocate(size_t size, size_t align) override;
        void do_deallocate(void* ptr, size_t size, size_t align) override;
        bool do_is_equal(const MemoryResource& other) const noexcept override;
};

}  // namespace memory
}  // namespace carbon
}  // namespace epic
