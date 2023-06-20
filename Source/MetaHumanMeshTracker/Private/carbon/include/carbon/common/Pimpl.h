// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <functional>

#include <carbon/common/Common.h>


namespace epic {
namespace carbon {
/**
    DLL compliant Pimpl smart pointer type

    Using std::unique_ptr<> for Pimpl pattern causes MSVC to report
    warnings for classes that should have a DLL interface. Therefore
    we introduce this class, to overcome this issue and offer us
    full control over DLL exporting.
*/
template<class T>
struct EPIC_CARBON_API Pimpl {
    using DeallocFn = void (*)(void*);

    Pimpl() : m_ptr(nullptr), m_dealloc(::operator delete) {
    }

    Pimpl(T* ptr, DeallocFn dealloc = ::operator delete) : m_ptr(ptr), m_dealloc(dealloc) {
    }

    Pimpl(const Pimpl& cp) = delete;
    Pimpl(Pimpl&& mv) {
        this->m_ptr = mv.m_ptr;
        this->m_dealloc = mv.m_dealloc;
        mv.m_ptr = nullptr;
        mv.m_dealloc = ::operator delete;
    }

    ~Pimpl() {
        if (this->m_ptr) {
            this->m_dealloc(this->m_ptr);
        }

        this->m_ptr = nullptr;
        this->m_dealloc = ::operator delete;
    }

    Pimpl& operator=(Pimpl const&) = delete;
    Pimpl& operator=(Pimpl&& mv) {
        if (this != &mv) {
            this->m_dealloc(this->m_ptr);
            this->m_ptr = mv.m_ptr;
            mv.m_ptr = nullptr;
            mv.m_dealloc = ::operator delete;
        }
        return *this;
    }

    operator bool() {
        return static_cast<bool>(this->m_ptr);
    }

    T* operator->() {
        return m_ptr;
    }

    const T* operator->() const {
        return m_ptr;
    }

    T* Get() {
        return m_ptr;
    }

    const T* Get() const {
        return m_ptr;
    }

    void Reset(T* ptr, DeallocFn dealloc = ::operator delete) {
        this->m_dealloc(this->m_ptr);
        this->m_ptr = ptr;
        this->m_dealloc = dealloc;
    }

    private:
        T* m_ptr;  // contained value
        DeallocFn m_dealloc;  // deallocator pointer - explicitly defined to help deallocate the pointer in the correct dll/exe.

};
}
}
