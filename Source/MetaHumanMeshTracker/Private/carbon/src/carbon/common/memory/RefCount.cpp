// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/common/memory/RefCount.h>

namespace epic {
namespace carbon {
namespace memory {

RefCount::RefCount() noexcept = default;

RefCount::~RefCount() noexcept = default;

void RefCount::AddRef() noexcept{
    ++m_refCount;
}

std::int32_t RefCount::Release() noexcept{
    return --m_refCount;
}

}  // namespace memory
}  // namespace carbon
}  // namespace epic
