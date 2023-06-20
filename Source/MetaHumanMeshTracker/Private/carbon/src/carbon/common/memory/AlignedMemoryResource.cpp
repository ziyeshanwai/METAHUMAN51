// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/common/memory/AlignedMemoryResource.h>

namespace epic {
namespace carbon {
namespace memory {

AlignedMemoryResource::AlignedMemoryResource(const memory_context& context) noexcept : m_context{context} {
}

AlignedMemoryResource::~AlignedMemoryResource() noexcept = default;

void* AlignedMemoryResource::do_allocate(size_t size, size_t  /*unused*/) {
    void* ptr{};
    #ifdef _WIN32
        ptr = _aligned_malloc(size, m_context.alignment);
    #else
        auto unused = posix_memalign(&ptr, m_context.alignment, size);
        (void)unused;
    #endif
    return ptr;
}

void AlignedMemoryResource::do_deallocate(void* ptr, size_t  /*unused*/, size_t  /*unused*/) {
    #ifdef _WIN32
        _aligned_free(ptr);
    #else
        std::free(ptr);
    #endif
}

bool AlignedMemoryResource::do_is_equal(const MemoryResource& other) const noexcept{
    return this == &other;
}

}  // namespace memory
}  // namespace carbon
}  // namespace epic
