// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <memory>

namespace epic::carbon {

/**
 * Most basic const-forwarding pointer wrapper for use with the Pimpl programming technique.
 * https://en.cppreference.com/w/cpp/language/pimpl.
 *
 * Use a custom delete Deleter = void(*)(T*) when the class does not have a destructor.
 * E.g. in the cpp class use Pimpl(std::unique_ptr<T, void(*)(T*)>(new T, [](T* ptr) {std::default_delete<T>()(ptr);}))
 */
template <class T, class Deleter = std::default_delete<T>>
class Pimpl
{
public:
    Pimpl(std::unique_ptr<T, Deleter>&& obj) : m(std::move(obj)) {}
    ~Pimpl() = default;
    Pimpl(Pimpl&&) = default;
    Pimpl(const Pimpl&) = delete;
    Pimpl& operator=(Pimpl&&) = default;
    Pimpl& operator=(const Pimpl&) = delete;

    constexpr T* operator->() { return m.get(); }
    constexpr const T* operator->() const { return m.get(); }
    constexpr T& operator*() { return *m.get(); }
    constexpr const T& operator*() const { return *m.get(); }
private:
    std::unique_ptr<T, Deleter> m;
};

} // namespace epic::carbon
