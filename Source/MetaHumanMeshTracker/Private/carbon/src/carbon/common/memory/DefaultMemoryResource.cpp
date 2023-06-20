// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/common/memory/DefaultMemoryResource.h>

namespace epic {
namespace carbon {
namespace memory {

DefaultMemoryResource::DefaultMemoryResource() noexcept = default;

DefaultMemoryResource::~DefaultMemoryResource() noexcept = default;

void* DefaultMemoryResource::do_allocate(size_t size, size_t  /*unused*/) {
    return std::malloc(size);
}

void DefaultMemoryResource::do_deallocate(void* ptr, size_t  /*unused*/, size_t  /*unused*/) {
    std::free(ptr);
}

bool DefaultMemoryResource::do_is_equal(const MemoryResource& other) const noexcept{
    return this == &other;
}

}  // namespace memory
}  // namespace carbon
}  // namespace epic
