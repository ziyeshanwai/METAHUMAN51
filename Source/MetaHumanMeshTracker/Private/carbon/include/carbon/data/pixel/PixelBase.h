// Copyright Epic Games, Inc. All Rights Reserved.

/*
    Basic / abstract pixel representation.

    This is an internal header, and it is not meant for API use.
    It is only included through carbon/data/Pixel.h file.
*/


#pragma once

#ifndef _CARBON_PIXEL_IS_INCLUDED
    #error "This is an implementation header of <carbon/data/Pixel.h> and is not supposed to be included on its own."
#endif

namespace epic::carbon {


template<class T>
struct EPIC_CARBON_API is_valid_type {
    static constexpr bool value = std::is_same<T, std::uint8_t>::value ||
        std::is_same<T, std::uint16_t>::value ||
        std::is_same<T, std::int16_t>::value  ||
        std::is_same<T, std::uint32_t>::value ||
        std::is_same<T, std::int32_t>::value  ||
        std::is_same<T, float>::value ||
        std::is_same<T, double>::value;
};
/**
    Main purpose of Pixel class is interaction with Image{T} class.

    It stores values at different channel related to particular pixel.

    @tparam T Type of data stored in Pixel class.
    @tparam size Dimensionality of pixel. Must be in range [1, 4]
*/
template<typename T, std::size_t size>
class EPIC_CARBON_API Pixel {
    static_assert(size > 0 && size <= 4, "Invalid pixel size - should be in [1, 4] range.");
    static_assert(is_valid_type<T>::value, "Invalid type provided for pixel.");

    public:
        using pixel_type = T;
        using reference = T&;
        using const_reference = const T&;
        using size_type = std::size_t;

        enum {
            channels = size
        };

        template<typename U, std::size_t sz>
        friend class Pixel;

        /**
        Default constructor.
        */
        Pixel() noexcept {
            static_assert(size > 0 && size < 5, "Invalid pixel size . Should be in range [1,4]");
            for (std::size_t i = 0; i < size; ++i) {
                m_elements[i] = T(0);
            }
        }

        /**
        Constructor.

        @param[in] val1 input value.
        */
        template<typename U, typename std::enable_if<std::is_convertible<U, T>::value, int>::type = 0>
        explicit Pixel(const U val1) noexcept {
            static_assert(size == 1, "Invalid constructor used as pixel size is different than 1.");
            m_elements[0] = T(val1);
        }

        /**
        Constructor.

        @param[in] val1 input value.
        @param[in] val2 input value.
        */
        template<typename U, typename std::enable_if<std::is_convertible<U, T>::value, int>::type = 0>
        explicit Pixel(const U val1, const U val2) noexcept {
            static_assert(size == 2, "Invalid constructor used as pixel size is different than 2.");
            m_elements[0] = T(val1);
            m_elements[1] = T(val2);
        }

        /**
        Constructor.

        @param[in] val1 input value.
        @param[in] val2 input value.
        @param[in] val3 input value.
        */
        template<typename U, typename std::enable_if<std::is_convertible<U, T>::value, int>::type = 0>
        explicit Pixel(const U val1, const U val2, const U val3) noexcept {
            static_assert(size == 3, "Invalid constructor used as pixel size is different than 3.");
            m_elements[0] = T(val1);
            m_elements[1] = T(val2);
            m_elements[2] = T(val3);
        }

        /**
        Constructor.

        @param[in] val1 input value.
        @param[in] val2 input value.
        @param[in] val3 input value.
        @param[in] val4 input value.
        */
        template<typename U, typename std::enable_if<std::is_convertible<U, T>::value, int>::type = 0>
        explicit Pixel(const U val1, const U val2, const U val3, const U val4) noexcept {
            static_assert(size == 4, "Invalid constructor used as pixel size is different than 4.");
            m_elements[0] = T(val1);
            m_elements[1] = T(val2);
            m_elements[2] = T(val3);
            m_elements[3] = T(val4);
        }

        /*
        Destructor.
        */
        ~Pixel() noexcept = default;

        /*
        Subscript operator - change.

        @param[in] idx index of pixel channel
        @return value of particular channel in pixel.
        */
        reference operator[](const size_type idx) {
            CARBON_PRECONDITION(idx < size, "Invalid index passed to operator[]");
            return m_elements[idx];
        }

        /*
        Subscript operator - view.

        @param[in] idx index of pixel channel
        @return value of particular channel in pixel.
        */
        const_reference operator[](const size_type idx) const {
            CARBON_PRECONDITION(idx < size, "Invalid index passed to operator[]");
            return m_elements[idx];
        }

        /*
        Parenthesis operator - change.

        @param[in] idx index of pixel channel
        @return value of particular channel in pixel.
        */
        reference operator()(const size_type idx) {
            CARBON_PRECONDITION(idx < size, "Invalid index passed to operator[]");
            return m_elements[idx];
        }

        /*
        Parenthesis operator - view.

        @param[in] idx index of pixel channel
        @return value of particular channel in pixel.
        */
        const_reference operator()(const size_type idx) const {
            CARBON_PRECONDITION(idx < size, "Invalid index passed to operator[]");
            return m_elements[idx];
        }

        /*
        Gets pixel channel count.

        @return pixel channel count.
        */
        constexpr size_type Channels() const {
            return static_cast<size_type>(channels);
        }

        /*
        Equality operator. Check whether lhs and rhs objects are equal according to implemented logic.

        @param[in] lhs left side operand.
        @param[in] rhs right side operand.

        @return true if lhs and rhs are equal, false otherwise
        */
        template<class U, std::size_t sz>
        bool operator==(const Pixel<U, sz>& rhs) const {
            static_assert(std::is_convertible<T, U>::value, "Pixel value types are not comparable (convertible).");
            if constexpr (size != sz) {
                return false;
            }
            for (std::size_t i = 0; i < size; ++i) {
                if (m_elements[i] != rhs[i]) {
                    return false;
                }
            }
            return true;
        }

        /*
        Inequality operator. Check whether lhs and rhs objects are not equal according to implemented logic.

        @param[in] lhs left side operand.
        @param[in] rhs right side operand.

        @return true if lhs and rhs are not equal, false otherwise
        */
        template<class U, std::size_t sz>
        bool operator!=(const Pixel<U, sz>& rhs) const {
            return !(*this == rhs);
        }

    protected:
        T m_elements[size];
};

}  // namespace epic::carbon
