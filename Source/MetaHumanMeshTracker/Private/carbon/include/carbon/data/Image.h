// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/common/Common.h>
#include <carbon/common/EigenDenseBackwardsCompatible.h>
#include <cstdint>
#if defined(__APPLE__) && __APPLE__
    #include <experimental/memory_resource>
#else
    #include <memory_resource>
#endif

#ifdef CARBON_USE_EIGEN_TENSOR
    #pragma warning(push)
    #pragma warning(disable:4554)
    #include <unsupported/Eigen/CXX11/Tensor>
    #pragma warning(pop)
#endif

#ifdef CARBON_USE_OPENCV_MAT
    #include <opencv2/core/mat.hpp>
#endif


#include <carbon/common/memory/DefaultMemoryResource.h>
#include <carbon/common/memory/AlignedMemoryResource.h>
#include <carbon/common/memory/RefCount.h>
#include <carbon/data/Pixel.h>

namespace epic {
namespace carbon {

#if defined(__APPLE__) && __APPLE__
    template<class T>
    using PolyAllocator = std::experimental::pmr::polymorphic_allocator<T>;
    using MemoryResource = std::experimental::pmr::memory_resource;
#else
    template<class T>
    using PolyAllocator = std::pmr::polymorphic_allocator<T>;
    using MemoryResource = std::pmr::memory_resource;
#endif

using epic::carbon::memory::AlignedMemoryResource;

using epic::carbon::memory::DefaultMemoryResource;

using carbon::memory::RefCount;

enum class ImageType : std :: uint8_t {
    Undefined = 0,
    Byte,
    UShort,
    SShort,
    UInt,
    SInt,
    Float,
    Double,
    Pixel
};

template<class T, class E = void>
struct ImageTraits;

// scalar representation, not a pixel type
template<class T>
struct ImageTraits<T, typename std::enable_if<!is_pixel<T>::value>::type> {
    using pixel_type = T;
    using value_type = typename PixelTraits<T>::value_type;
    using matrix_element_type = T;
    using matrix_type = Eigen::Array<T, -1, -1, Eigen::RowMajor>;

    constexpr static ColorSpace color_space = PixelTraits<T>::color_space;

    enum {
        channels = PixelTraits<T>::channels
    };
};

// pixel type
template<class T>
struct ImageTraits<T, typename std::enable_if<is_pixel<T>::value && PixelTraits<T>::channels == 1>::type> {
    using pixel_type = T;
    using value_type = typename PixelTraits<T>::value_type;
    using matrix_element_type = typename PixelTraits<T>::value_type;
    using matrix_type = Eigen::Array<matrix_element_type, -1, -1, Eigen::RowMajor>;

    constexpr static ColorSpace color_space = PixelTraits<T>::color_space;

    enum {
        channels = PixelTraits<T>::channels
    };
};

template<class T>
struct ImageTraits<T, typename std::enable_if<is_pixel<T>::value && PixelTraits<T>::channels != 1>::type> {
    using pixel_type = T;
    using value_type = typename PixelTraits<T>::value_type;
    using matrix_element_type = Eigen::Array<typename PixelTraits<T>::value_type, PixelTraits<T>::channels, 1>;
    using matrix_type = Eigen::Array<matrix_element_type, -1, -1, Eigen::RowMajor>;

    constexpr static ColorSpace color_space = PixelTraits<T>::color_space;

    enum {
        channels = PixelTraits<T>::channels
    };
};

template<class T, class U>
struct AreCompatibleTypes {
    constexpr static bool value = (is_valid_type<T>::value && is_pixel<U>::value) ||
        (is_valid_type<U>::value && is_pixel<T>::value);
};

/*
@Brief Image class represents rudimentary wrapper around raw image data.

@tparam type of data stored in image.
<bt>Supported are:
- std::int8_t
- std::unit16_t
- std::int16_t
- std::uint32_t
- std::int32_t
- float
- double
- Pixel{T} of allowed primitive types

@note
    Image class either owns its memory or represents a view over an external data.

@remarks
    Current choice is templated image, but it could evolve to runtime type storage.
*/
template<class T>
class EPIC_CARBON_API Image {
    public:
        using value_type = typename ImageTraits<T>::value_type;
        using pixel_type = typename ImageTraits<T>::pixel_type;
        using matrix_type = typename ImageTraits<T>::matrix_type;
        using pointer = T*;
        using const_pointer = const T*;
        using size_type = std::size_t;
        using reference = T&;
        using const_reference = const T&;

        #ifdef CARBON_USE_EIGEN_TENSOR
            using tensor_type = Eigen::Tensor<value_type, 3, Eigen::RowMajor>;
        #endif

        enum {  // propagate this trait to image for ease of use in templates
            channels = ImageTraits<T>::channels
        };

        // ! For now, lets using plain pointers as iterators
        using const_iterator = const_pointer;
        using iterator = pointer;
        #if defined(__APPLE__) && __APPLE__
            using allocator_type = std::experimental::pmr::polymorphic_allocator<value_type>;
            using allocator_type_ref_count = std::experimental::pmr::polymorphic_allocator<RefCount>;
        #else
            using allocator_type = std::pmr::polymorphic_allocator<value_type>;
            using allocator_type_ref_count = std::pmr::polymorphic_allocator<RefCount>;
        #endif

        template<class U>
        friend class Image;

        /// Default constructor
        Image() noexcept {
        }

        /// Allocate an empty (uninitialized image) of given size.
        Image(size_type rows, size_type cols, MemoryResource* memRes = nullptr)  :
            m_rows{rows},
            m_cols{cols},
            m_size{rows* cols},
            m_strides{cols, 1},
            m_ownsData{true} {

            if (!memRes) {
                static DefaultMemoryResource dmr;
                m_memoryResource = &dmr;
            } else {
                m_memoryResource = memRes;
            }

            allocator_type polyAlocator{m_memoryResource};
            m_data = m_dataBegin = reinterpret_cast<pixel_type*>(polyAlocator.allocate(m_size * channels));
            m_refCounting = new RefCount();
            m_refCounting->AddRef();
        }

        /**
         Copy constructor.

         @note
             During copy construction, shallow copy is created i.e. no data copy is performed.
             <br>Also, reference counter is increased.
             <br>In order to obtain deep copy, Clone method should be used.
        */
        Image(const Image<T>& other) {
            *this = other;
        }

        /**
            Constructor for construction of Image class from raw data.

            @param[in] data input pixel buffer.
            @param[in] rows number of rows, ie. the height of the image.
            @param[in] cols number of columns, ie. the width of the image.
            @param[in] dataBegin Pointer where the data of this image begins.
            @param[in] strideRows Stride count for indexing rows (how many elements you need to jump to get to another row).
            @param[in] strideCols Stride count for indexing columns (how many elements you need to jump to get to another column).
            @param[in] ownsData whether deep copy should be created or view mode is active i.e. whether or not this image is data owner.
            @param[in] memRes MemoryResource object used in poly allocator for object allocation/deallocation

            @note
                ownsData = 1 means that deep copy of the data should be created upon Image{T} object creation and that reference counting could be turned on.
                ownsData = 0 should be set if this image object is intended for view mode.
        */
        Image(pointer data,
              pointer dataBegin,
              const size_type rows,
              const size_type cols,
              const size_type strideRows,
              const size_type strideCols,
              const bool ownsData = true,
              MemoryResource* memRes = nullptr) :
            m_rows{rows},
            m_cols{cols},
            m_size{rows* cols},
            m_strides{strideRows, strideCols},
            m_ownsData{ownsData} {

            CARBON_PRECONDITION(m_dataBegin >= m_data,
                                "Invalid data - memory pointer does not equal or precede data begin pointer.");

            CARBON_PRECONDITION(rows && cols, "Invalid sizes - should be larger than zero.");

            if (!memRes) {
                static DefaultMemoryResource dmr;
                m_memoryResource = &dmr;
            } else {
                m_memoryResource = memRes;
            }

            if (!ownsData) {
                m_data = data;
                m_dataBegin = dataBegin;
            } else {
                allocator_type polyAllocator{m_memoryResource};
                m_data = m_dataBegin = reinterpret_cast<pixel_type*>(polyAllocator.allocate(m_size * channels));

                if ((strideRows == cols) && (strideCols == 1)) {  // if input data is contiguous
                    std::memcpy(m_data, data, m_size * sizeof(pixel_type));
                } else {
                    // copy with respect to the input striding
                    for (size_type r = 0; r < rows; ++r) {
                        for (size_type c = 0; c < cols; ++c) {
                            polyAllocator.construct(m_data + r * cols + c, dataBegin[r * strideRows + c * strideCols]);
                        }
                    }
                }

                m_refCounting = new RefCount();
                m_refCounting->AddRef();
            }

        }

        /*
        Constructor for construction of Image class from raw data.

        Convenience function which assumes data is contiguous, therefore
        bypasses data-begin pointer and stride values.

        @param[in] data input pixel buffer.
        @param[in] rows number of rows, ie. the height of the image.
        @param[in] cols number of columns, ie. the width of the image.
        @param[in] ownsData whether deep copy should be created or view mode is active i.e. whether or not this image is data owner.
        @param[in] memRes MemoryResource object used in poly allocator for object allocation/deallocation

        @note
            ownsData = 1 means that deep copy of the data should be created upon Image{T} object creation and that reference counting could be turned on.
            ownsData = 0 should be set if this image object is intended for view mode.
        */
        Image(pointer data,
              const size_type rows,
              const size_type cols,
              const bool ownsData = true,
              MemoryResource* memRes = nullptr) :
            Image(data, data, rows, cols, cols, 1, ownsData, memRes) {
        }

        /**
         Assignment operator.

         @note
             During assignment operation, shallow copy is created i.e. no data copy is performed.
             In order to obtain deep copy, Clone() method should be used.
             <br>Also, reference counter is increased.
        */
        Image<T>& operator=(const Image<T>& other) {
            if (this != &other) {
                CopyInternal(other);

                if (m_ownsData) {
                    m_refCounting->AddRef();
                }
            }
            return *this;
        }

        /**
         Move constructor.
        */
        Image(Image<T>&& other) noexcept {
            MoveInternal(other);
        }

        /**
         Move operator.
        */
        Image<T>& operator=(Image<T>&& other) noexcept {
            MoveInternal(other);
            return *this;
        }

        /**
         Destructor.
        */
        ~Image() noexcept {
            // During move operation m_refCounting is destroyed.
            if (m_ownsData && m_refCounting && !m_refCounting->Release()) {
                allocator_type polyAlocator{m_memoryResource};
                polyAlocator.deallocate(reinterpret_cast<value_type*>(m_data), 0);
                delete m_refCounting;
            }
            m_size = m_rows = m_cols = m_strides[0] = m_strides[1] = size_type(0);
            m_data = m_dataBegin = nullptr;
            m_refCounting = nullptr;
        }

        /// Check if this image object owns (and therefore deallocates) the data it contains.
        bool OwnsData() const noexcept {
            return m_ownsData;
        }

        /// Check if this image object is empty (no data contained).
        bool IsEmpty() const noexcept {
            return m_data == nullptr;
        }

        /// Check if this image memory is contiguous (each next row and column start right after the previous).
        bool IsContiguous() const noexcept {
            return m_strides[0] == m_cols &&
                   m_strides[1] == 1;
        }

        /// Check if this image is a slice (ROI) of some other image.
        bool IsSlice() const noexcept {
            return (m_data != m_dataBegin) || !IsContiguous();
        }

        /**
        Gets the image width i.e. number of pixels in one image column.
        @return image width
        */
        size_type Cols() const noexcept {
            return m_cols;
        }

        /**
        Gets the image height i.e. number of pixels in one image row.
        @return image height
        */
        size_type Rows() const noexcept {
            return m_rows;
        }

        /**
        Gets number of image channels.

        @note This is a wrapper over static call Image<T>::channels,
            which should be preferred to this wherever possible.

        @return number of channels
        */
        size_type Channels() const noexcept {
            return !IsEmpty() ? Image<T>::channels : 0;
        }

        /**
            Gets image stride values.
            @return image strides for rows and columns.
        */
        const size_type* Strides() const noexcept {
            return m_strides;
        }

        /**
        Gets the image size.
        @return image size

        @note
            image size is equal to width \cdot height.
        */
        size_type Size()  const noexcept {
            return m_size;
        }

        /**
            Gets the image buffer (memory) size.
            @return image buffer size

            @note
                image size is equal to width \cdot height \cdot channels \cdot sizeof(Image<>::value_type)

            @warning
                This function returns plain buffer size required to hold current image, which may be untrue
                if image data buffer is not contiguous. Use Image::IsContiguous to make sure internal buffer
                is safe to handle.
        */
        size_type BufferSize()  const noexcept {
            return m_size * sizeof(T);
        }

        /**
            Gets the internal image buffer.
            @return internal buffer

            @warning
                Handle carefully, and always check if internal memory buffer is contiguous before handling
                raw data.
        */
        pointer Data() noexcept {
            return m_dataBegin;
        }

        /**
            Gets the internal image buffer.
            @return internal buffer

            @warning
                Handle carefully, and always check if internal memory buffer is contiguous before handling
                raw data.
        */
        const_pointer Data() const noexcept {
            return m_dataBegin;
        }

        /**
            Image slicing operation.

            @param[in] rfrom Starting row from which image is sliced.
            @param[in] rto Ending row to which image is sliced.
            @param[in] rstride How many rows we jump to reach the next row.
            @param[in] cfrom Starting column from which image is sliced.
            @param[in] cto Ending columns to which image is sliced.
            @param[in] cstride How many columns we jump to reach the next columns.
        */
        Image<T> Slice(size_type rfrom, size_type rto, size_type rstride, size_type cfrom, size_type cto, size_type cstride) {
            return SliceInternal(rfrom, rto, rstride, cfrom, cto, cstride);
        }

        const Image<T> Slice(size_type rfrom, size_type rto, size_type rstride, size_type cfrom, size_type cto,
                             size_type cstride) const {
            return SliceInternal(rfrom, rto, rstride, cfrom, cto, cstride);
        }

        /// Convenience slicing where strides are 1.
        Image<T> Slice(size_type rfrom, size_type rto, size_type cfrom, size_type cto) {
            return this->Slice(rfrom, rto, 1, cfrom, cto, 1);
        }

        const Image<T> Slice(size_type rfrom, size_type rto, size_type cfrom, size_type cto) const {
            return this->Slice(rfrom, rto, 1, cfrom, cto, 1);
        }

        /// Convenience slicing only by rows.
        Image<T> RowSlice(size_type from, size_type to, size_type stride = 1) {
            return this->Slice(from, to, stride, 0, m_cols, 1);
        }

        const Image<T> RowSlice(size_type from, size_type to, size_type stride = 1) const {
            return this->Slice(from, to, stride, 0, m_cols, 1);
        }

        /// Convenience slicing only by columns.
        Image<T> ColSlice(size_type from, size_type to, size_type stride = 1) {
            return this->Slice(0, m_rows, 1, from, to, stride);
        }

        const Image<T> ColSlice(size_type from, size_type to, size_type stride = 1) const {
            return this->Slice(0, m_rows, 1, from, to, stride);
        }

        /**
            Slice a window in this image.

            @param[in] row Row at which pixel around which window is sliced is located.
            @param[in] col Column at which pixel around which window is sliced is located.
            @param[in] rows How many rows does the window span.
            @param[in] cols How many columns does the window span.

            Slice a window around selected pixel, of given size. For e.g given the following
            (grayscale) image matrix:

            @code
            1  2  3  4  5
            6  7  8  9  10
            11 12 13 14 15
            16 17 18 19 20
            21 22 23 24 25
            @endcode

            Following window slicing:

            @code
            image.Window(2, 2, 3);
            @endcode

            ... would slice the following sub-image:

            @code
            7  8  9
            12 13 14
            17 18 19
            @endcode

            @return
                Window image, a reference counted slice.
        */
        Image<T> Window(size_type row, size_type col, size_type rows, size_type cols) {
            CARBON_PRECONDITION(rows >= 1 && cols >= 1, "Window rows (height) and cols (width) have to be above zero.");
            CARBON_PRECONDITION(rows % 2 && cols % 2, "Rows and cols have to be odd.");

            const int startRow = static_cast<int>(row) - static_cast<int>(rows) / 2;
            const int endRow = static_cast<int>(row) + static_cast<int>(rows) / 2 + 1;
            const int startCol = static_cast<int>(col) - static_cast<int>(cols) / 2;
            const int endCol = static_cast<int>(col) + static_cast<int>(cols) / 2 + 1;

            CARBON_PRECONDITION(startRow > 0 && startRow <= m_rows &&
                                startCol > 0 && startCol <= m_cols,
                                "Window out of bounds.");

            return this->Slice(startRow, endRow, startCol, endCol);
        }

        /// Convenience window slicing for rectangular windows.
        Image<T> Window(size_type row, size_type col, size_type size) {
            return Window(row, col, size, size);
        }

        /**
            Create an image of given dimensions that is filled with constant valued pixels.
        */
        static Image<T> Constant(size_type rows, size_type cols, T value) {
            CARBON_PRECONDITION(rows && cols, "Given dimensions must be greater than 0.");
            Image<T> img(rows, cols);
            std::for_each(img.begin(), img.end(), [&value](T& pix) {
                    new (&pix) T(value);
                });
            return img;
        }

        /**
            Create an image of given dimensions that is filled with 0-valued pixels.
        */
        static Image<T> Zero(size_type rows, size_type cols) {
            return Image<T>::Constant(rows, cols, epic::carbon::Zero<T>());
        }

        /**
            Create an image of given dimensions that is filled with 0-valued pixels.
        */
        static Image<T> Ones(size_type rows, size_type cols) {
            return Image<T>::Constant(rows, cols, epic::carbon::Ones<T>());
        }

        /**
        Gets matrix representation of image.

        Maps the image data to Eigen::Matrix, where
        type of data is type of image pixel.

        @note Returned matrix is row-major.

        @return Eigen::Map for the Matrix representation of an image.
        */
        Eigen::Map<const matrix_type> ToMatrix() const {
            CARBON_PRECONDITION(!IsEmpty(), "Image must not be empty for ToMatrix call.");
            CARBON_PRECONDITION(IsContiguous(),
                                "This image is not contiguous - first clone to new memory before mapping to Eigen::Matrix.");

            return {reinterpret_cast<typename ImageTraits<T>::matrix_element_type*>(m_data),
                    static_cast<Eigen::Index>(m_rows),
                    static_cast<Eigen::Index>(m_cols)};
        }

        /**
            Non-const variant for mapping image to matrix.
        */
        Eigen::Map<matrix_type> ToMatrix() {
            CARBON_PRECONDITION(!IsEmpty(), "Image must not be empty for ToMatrix call.");
            CARBON_PRECONDITION(IsContiguous(),
                                "This image is not contiguous - first clone to new memory before mapping to Eigen::Matrix.");

            return {reinterpret_cast<typename ImageTraits<T>::matrix_element_type*>(m_data),
                    static_cast<Eigen::Index>(m_rows),
                    static_cast<Eigen::Index>(m_cols)};
        }

        /**
        Maps an Eigen::Matrix to image type.

        @return Image object that holds the data and properties of given matrix, but does not own the data.
        */
        static const Image<T> FromMatrix(const matrix_type& matrix) {
            return {reinterpret_cast<pointer>(const_cast<typename ImageTraits<T>::matrix_element_type*>(matrix.data())),
                    static_cast<size_type>(matrix.rows()),
                    static_cast<size_type>(matrix.cols()),
                    false};  // ownsData
        }

        static Image<T> FromMatrix(matrix_type& matrix) {
            return {reinterpret_cast<pointer>(matrix.data()),
                    static_cast<size_type>(matrix.rows()),
                    static_cast<size_type>(matrix.cols()),
                    false};  // ownsData
        }

        #ifdef CARBON_USE_EIGEN_TENSOR
            /**
            Gets tensor (rank 3: rows, cols, channels) representation of image.

            Maps the image data to Eigen::Tensor, where
            type of data is type of image pixel.

            @note Returned tensor is row-major.

            @return Eigen::TensorMap for the Tensor representation of an image.
            */
            Eigen::TensorMap<const tensor_type> ToTensor() const {
                CARBON_PRECONDITION(!IsEmpty(), "Image must not be empty for ToTensor call.");
                CARBON_PRECONDITION(IsContiguous(),
                                    "This image is not contiguous - first clone to new memory before mapping to Eigen::Tensor.");

                return {
                    reinterpret_cast<value_type*>(m_data),
                    static_cast<Eigen::Index>(m_rows),
                    static_cast<Eigen::Index>(m_cols),
                    static_cast<Eigen::Index>(channels)};
            }

            /**

            */
            static const Image<T> FromTensor(const tensor_type& tensor) {
                CARBON_PRECONDITION(tensor.size() != 0, "Tensor must not be empty for FromTensor call.");
                CARBON_PRECONDITION(tensor.dimension(2) == channels,
                                    "Invalid tensor provided - channel count does not correspond to image type.");
                return {reinterpret_cast<pointer>(const_cast<value_type*>(&tensor(0, 0, 0))),
                        static_cast<size_type>(tensor.dimension(0)),
                        static_cast<size_type>(tensor.dimension(1)),
                        false};  // ownsData
            }

            static Image<T> FromTensor(tensor_type& tensor) {
                CARBON_PRECONDITION(tensor.size() != 0, "Tensor must not be empty for FromTensor call.");
                CARBON_PRECONDITION(tensor.dimension(2) == channels,
                                    "Invalid tensor provided - channel count does not correspond to image type.");
                return {reinterpret_cast<pointer>(&tensor(0, 0, 0)),
                        static_cast<size_type>(tensor.dimension(0)),
                        static_cast<size_type>(tensor.dimension(1)),
                        false};  // ownsData
            }

        #endif

        #ifdef CARBON_USE_OPENCV_MAT
            /**
                Encapsulates image data temporarily inside cv::Mat structure.

                @return cv::Mat structure of the same properties, containing
                    the same data from this image.

                @note This function is not defined for data types that are not
                    supported by OpenCV. The list of supported data types is:
                    unsigned char, bool, signed char, unsigned short, signed short,
                    int, float, double.

                @warning As returned, cv::Mat object does not contain ownership,
                    carbon::Image object that is used to create it has to be
                    kept in life as long as cv::Mat object is required. Otherwise
                    create a deep clone of cv::Mat object to keep using it.
            */
            cv::Mat ToCvMat() const {
                CARBON_PRECONDITION(m_strides[1] == 1, "Image columns should be contiguous to be convertible to cv::Mat.");

                if (IsEmpty()) {
                    return cv::Mat();
                }
                return cv::Mat(
                    static_cast<int>(m_rows),
                    static_cast<int>(m_cols),
                    CV_MAKE_TYPE(cv::traits::Type<value_type>::value, static_cast<int>(channels)),
                    reinterpret_cast<void*>(m_dataBegin),
                    m_strides[0] * sizeof(T));
            }

            /**
                Creates an Image out of a cv::Mat object, without the ownership of the data.

                @return Image structure of the same properties, containing
                    the same data from given cv::Mat object.

                @warning Because carbon::Image only support contiguous data layout, if
                    passed cv::Mat is incontiguous (strided), exception is thrown.

                @warning As returned, carbon::Image object does not contain ownership,
                    cv::Mat object that is used to create it has to be
                    kept in life as long as carbon::Image object is required. Otherwise
                    create a deep clone of carbon::Image object to keep using it.
            */
            static Image<T> FromCvMat(const cv::Mat& mat) {
                if (!mat.data) {
                    return Image<T>();
                }
                CARBON_PRECONDITION(mat.flags & cv::Mat::CONTINUOUS_FLAG, "Provided cv::Mat data is not continuous.");
                CARBON_PRECONDITION(
                    mat.channels() == Image<T>::channels,
                    "Provided cv::Mat object does not correspond to Image<T> definition - channel count is not the same.");

                return {reinterpret_cast<pointer>(const_cast<unsigned char*>(mat.datastart)),
                        reinterpret_cast<pointer>(const_cast<unsigned char*>(mat.data)),
                        static_cast<size_type>(mat.rows),
                        static_cast<size_type>(mat.cols),
                        static_cast<size_type>(mat.step[0] / sizeof(T)),  // strideRows
                        static_cast<size_type>(1),  // strideCols
                        false,  // ownsData
                        nullptr};
            }

        #endif

        /*
        Gives iterator positioned at the beginning of the image.

        @return iterator at the beginning of the image.
        */
        iterator begin() {
            CARBON_PRECONDITION(IsContiguous(), "Only contiguous images could be iterated over.");
            return m_data;
        }

        /*
        Gives iterator positioned at the end of the image.

        @return iterator at the end of the image.
        */
        iterator end() {
            CARBON_PRECONDITION(IsContiguous(), "Only contiguous images could be iterated over.");
            return m_data + m_size;
        }

        /*
        Gives constant iterator positioned at the beginning of the image.

        @return constant iterator at the beginning of the image.
        */
        const_iterator cbegin() const {
            CARBON_PRECONDITION(IsContiguous(), "Only contiguous images could be iterated over.");
            return m_data;
        }

        /*
        Gives constant iterator positioned at the end of the image.

        @return constant iterator at the end of the image.
        */
        const_iterator cend() const {
            CARBON_PRECONDITION(IsContiguous(), "Only contiguous images could be iterated over.");
            return m_data + m_size;
        }

        /*
        Gives constant iterator positioned at the beginning of the image.

        @return constant iterator at the beginning of the image.
        */
        const_iterator begin() const {
            return cbegin();
        }

        /*
        Gives constant iterator positioned at the end of the image.

        @return constant iterator at the end of the image.
        */
        const_iterator end() const {
            return cend();
        }

        /**
        Performs deep copying of an image. Reference counting of new image stars at 0.

        @returns new image that contains copy of data.
        */
        Image<T> Clone() const {
            return {
                m_data,
                m_dataBegin,
                m_rows,
                m_cols,
                m_strides[0],
                m_strides[1],
                true,  // ownsData
                m_memoryResource
            };
        }

        /**
            Perform image type / color conversion.
        */
        template<class U, typename std::enable_if<!std::is_same<U, T>::value, int>::type = 0>
        Image<typename std::remove_cv<U>::type> ConvertTo() const {
            using U_ = typename std::remove_cv<U>::type;

            static_assert(is_pixel_convertible<T, U_>::value, "Requested conversion is not supported.");
            Image<U_> out(m_rows, m_cols);
            out.PixelTransform(*this, [](const T& pixel) -> U_ {
                    return static_cast<U_>(pixel);
                });

            return out;
        }

        template<class U, typename std::enable_if<std::is_same<U, T>::value, int>::type = 0>
        Image<typename std::remove_cv<U>::type> ConvertTo() const {
            return this->Clone();
        }

        /*
        Access operator.
        @param[in] index pixel position in image.

        @returns reference to image pixel.
        */
        reference operator[](const size_type index) {
            CARBON_PRECONDITION(index < m_size, "Invalid index passed to operator[]");
            CARBON_PRECONDITION(IsContiguous(), "Only contiguous images can be accessed with a single index.");
            return m_data[index];
        }

        /*
        Access operator.
        @param[in] index pixel position in image.

        @returns constant reference to image pixel.
        */
        const_reference operator[](const size_type index) const {
            CARBON_PRECONDITION(index < m_size, "Invalid index passed to operator[]");
            CARBON_PRECONDITION(IsContiguous(), "Only contiguous images can be accessed with a single index.");
            return m_data[index];
        }

        /*
        Access operator - gives pixel element at given coordinates and at given channel.

        @param[in] row pixel position at y axis.
        @param[in] col pixel position at x axis.
        */
        pixel_type& operator()(const size_type row, const size_type col) {
            CARBON_PRECONDITION(row < m_rows, "Invalid row index passed to operator()");
            CARBON_PRECONDITION(col < m_cols, "Invalid col index passed to operator()");
            return *(m_dataBegin + (row * m_strides[0] + col * m_strides[1]));
        }

        /*
        Access operator - gives pixel value at given coordinates and at given channel.

        @param[in] row pixel position at y axis.
        @param[in] col pixel position at x axis.
        */
        const pixel_type& operator()(const size_type row, const size_type col) const {
            CARBON_PRECONDITION(row < m_rows, "Invalid row index passed to operator()");
            CARBON_PRECONDITION(col < m_cols, "Invalid col index passed to operator()");
            return *(m_dataBegin + (row * m_strides[0] + col * m_strides[1]));
        }

        /*
        Access operator - gives pixel value at given coordinates and at given channel.

        @param[in] row pixel position at y axis.
        @param[in] col pixel position at x axis.
        @param[in] channel chanel coordinate i.e. B/G/R
        */
        value_type& operator()(const size_type row, const size_type col, const size_type channel) {
            static_assert(!std::is_same<pixel_type, value_type>::value,
                          "Channel indexing is allowed only for Image<Pixel<T, N>> types.");
            CARBON_PRECONDITION(row < m_rows, "Invalid row index passed to operator()");
            CARBON_PRECONDITION(col < m_cols, "Invalid col index passed to operator()");
            CARBON_PRECONDITION(channel < channels, "Invalid channel index passed to operator()");
            return (*this)(row, col)[channel];
        }

        /*
        Access operator - gives pixel value at given coordinates and at given channel.

        @param[in] row pixel position at y axis.
        @param[in] col pixel position at x axis.
        @param[in] channel chanel coordinate i.e. B/G/R
        */
        const value_type& operator()(const size_type row, const size_type col, const size_type channel) const {
            static_assert(!std::is_same<pixel_type, value_type>::value,
                          "Channel indexing is allowed only for Image<Pixel<T, N>> types.");
            CARBON_PRECONDITION(row < m_rows, "Invalid row index passed to operator()");
            CARBON_PRECONDITION(col < m_cols, "Invalid col index passed to operator()");
            CARBON_PRECONDITION(channel < channels, "Invalid channel index passed to operator()");
            return (*this)(row, col)[channel];
        }

        /*
        Gives image type.

        @return type of the image.
        */
        constexpr ImageType GetType() const {
            if (is_pixel<T>::value) {
                return ImageType::Pixel;
            }

            if (std::is_same<typename Image<T>::value_type, std::uint8_t>::value) {
                return ImageType::Byte;
            }

            if (std::is_same<typename Image<T>::value_type, std::uint16_t>::value) {
                return ImageType::UShort;
            }

            if (std::is_same<typename Image<T>::value_type, std::int16_t>::value) {
                return ImageType::SShort;
            }

            if (std::is_same<typename Image<T>::value_type, std::uint32_t>::value) {
                return ImageType::UInt;
            }

            if (std::is_same<typename Image<T>::value_type, std::int32_t>::value) {
                return ImageType::SInt;
            }

            if (std::is_same<typename Image<T>::value_type, float>::value) {
                return ImageType::Float;
            }

            if (std::is_same<typename Image<T>::value_type, double>::value) {
                return ImageType::Double;
            }

            return ImageType::Undefined;
        }

        /**
            Collapse pixel type into primitive for single-channel images.

            For e.g. Image<Mono<float>> will be collapsed to Image<float>.

            @note this operation is done with reference counting i.e. shallow copy of the source one.
        */
        Image<value_type> CollapsePixels() {
            static_assert(channels == 1, "Pixel collapse allowed only for single-channel images.");
            CARBON_PRECONDITION(m_size, "Pixel collapse operation is not allowed on empty images.");
            Image<value_type> collapsed;
            collapsed.CopyInternal(*this);
            if (collapsed.m_ownsData) {
                collapsed.m_refCounting->AddRef();
            }
            return collapsed;
        }

        /// Read only variant for pixel collapsing.
        const Image<value_type> CollapsePixels() const {
            static_assert(channels == 1, "Pixel collapse allowed only for single-channel images.");
            CARBON_PRECONDITION(m_size, "Pixel collapse operation is not allowed on empty images.");
            Image<value_type> collapsed;
            collapsed.CopyInternal(*this);
            if (collapsed.m_ownsData) {
                collapsed.m_refCounting->AddRef();
            }
            return collapsed;
        }

        /**
            Pixel packing - opposite to collapse.

            For e.g. Image<float> will be packed into Image<Mono<float>>.

            @note this operation is done with reference counting i.e. shallow copy of the source one.
        */
        Image<Mono<T> > PackPixels() {
            static_assert(!is_pixel<T>::value, "Pixel packing is only allowed for scalar (primitive) images.");
            CARBON_PRECONDITION(m_size, "Pixel collapse operation is not allowed on empty images.");
            Image<Mono<T> > packed;
            packed.CopyInternal(*this);
            if (packed.m_ownsData) {
                packed.m_refCounting->AddRef();
            }
            return packed;
        }

        const Image<Mono<T> > PackPixels() const {
            static_assert(!is_pixel<T>::value, "Pixel packing is only allowed for scalar (primitive) images.");
            CARBON_PRECONDITION(m_size, "Pixel collapse operation is not allowed on empty images.");
            Image<Mono<T> > packed;
            packed.CopyInternal(*this);
            if (packed.m_ownsData) {
                packed.m_refCounting->AddRef();
            }
            return packed;
        }

        /*
        Conversion / cast operator - converts image to image of primitive allowed types.

        @return Image of allowed primitive type.

        @note
            Deep copy is always created when using this operator.
        */
        explicit operator Image<value_type>() const {
            static_assert(channels == 1, "Casting image to primitives is not allowed for multi-channel images.");
            return Image<value_type>(reinterpret_cast<value_type*>(m_data), m_rows, m_cols);
        }

        /**
            Visit each pixel with a provided unary function.

            @tparam UnaryFn Unary function that takes each pixel reference and performs required action / mutation.
        */
        template<class UnaryFn>
        void PixelVisit(UnaryFn f) {
            if (IsEmpty()) {
                return;
            }

            // dev note - we intentionally use raw data access to
            // avoid bound checking in index operators
            if (IsContiguous()) {
                for (size_type s = 0; s < m_size; ++s) {
                    f(m_dataBegin[s]);
                }
            } else {
                for (size_type r = 0; r < m_rows; ++r) {
                    for (size_type c = 0; c < m_cols; ++c) {
                        f(m_dataBegin[r * m_strides[0] + c * m_strides[1]]);
                    }
                }
            }
        }

        /**
            Transform this image pixel values using given image as starting value, and provided unary function.

            @param[in] image Starting pixel values. Should be of same size as current image.
            @param[in] f Unary functor taking input image pixel values and transforming them to this image pixel type.
            @tparam UnaryFn Unary pure function, that transform U value from input image to the result / output buffer: f(U) -> T
        */
        template<class U, class UnaryFn>
        void PixelTransform(const Image<U>& image, UnaryFn f) {
            static_assert(is_pixel_convertible<T, U>::value, "Provided image is not of convertible pixel type to this image.");
            CARBON_PRECONDITION(m_rows == image.Rows() && m_cols == image.Cols(), "Images should be of same size.");

            if (IsContiguous() && image.IsContiguous()) {
                for (size_type s = 0; s < m_size; ++s) {
                    m_dataBegin[s] = f(image.m_dataBegin[s]);
                }
            } else {
                for (size_type r = 0; r < m_rows; ++r) {
                    for (size_type c = 0; c < m_cols; ++c) {
                        const U& uval = image.m_dataBegin[r * image.m_strides[0] + c * image.m_strides[1]];
                        m_dataBegin[r * m_strides[0] + c * m_strides[1]] = f(uval);
                    }
                }
            }
        }

    private:
        template<class U>
        void CopyInternal(const Image<U>& other) {
            static_assert(sizeof(T) == sizeof(U),
                          "CARBON INTERNAL ERROR: Image::CopyInternal misused - different types do not match.");

            m_rows = other.m_rows;
            m_cols = other.m_cols;
            m_size = other.m_size;
            m_strides[0] = other.m_strides[0];
            m_strides[1] = other.m_strides[1];

            if (m_data && m_ownsData && !m_refCounting->Release()) {
                allocator_type polyAlocator {
                    m_memoryResource
                };
                polyAlocator.deallocate(reinterpret_cast<value_type*>(m_data), 0);
                delete m_refCounting;
            }

            m_data = reinterpret_cast<T*>(other.m_data);
            m_dataBegin = reinterpret_cast<T*>(other.m_dataBegin);
            m_ownsData = other.m_ownsData;
            m_refCounting = other.m_refCounting;
            m_memoryResource = other.m_memoryResource;
        }

        template<class U>
        void MoveInternal(Image<U>& other) {
            static_assert(sizeof(T) == sizeof(U),
                          "CARBON INTERNAL ERROR: Image::MoveInternal misused - different types do not match.");

            if (this != &other) {
                CopyInternal(other);
                other.m_refCounting = nullptr;
                other.m_data = other.m_dataBegin = nullptr;
                other.m_memoryResource = nullptr;
                other.m_rows = other.m_cols = other.m_size = other.m_strides[0] = other.m_strides[1] = size_type{};
            }
        }

        Image<T> SliceInternal(size_type rfrom,
                               size_type rto,
                               size_type rstride,
                               size_type cfrom,
                               size_type cto,
                               size_type cstride) const {
            CARBON_PRECONDITION(rfrom < rto, "Invalid slice - \"from\" should be lower than \"to\".");
            CARBON_PRECONDITION(cfrom < cto, "Invalid slice - \"from\" should be lower than \"to\".");
            CARBON_PRECONDITION(rfrom >= 0 && rfrom < m_rows, "Invalid \"from\" value - should be [0, rows).");
            CARBON_PRECONDITION(cfrom >= 0 && cfrom < m_cols, "Invalid \"from\" value - should be [0, cols).");

            CARBON_PRECONDITION(rstride < rto - rfrom, "Row striding value is too large - goes out of given bounds.");
            CARBON_PRECONDITION(cstride < cto - cfrom, "Column striding value is too large - goes out of given bounds.");

            const size_type beginOffset = m_strides[0] * rfrom + m_strides[1] * cfrom;

            Image<T> slice;

            slice.m_data = m_data;
            slice.m_ownsData = m_ownsData;
            slice.m_refCounting = m_refCounting;

            if (m_ownsData) {
                slice.m_refCounting->AddRef();
            }

            slice.m_dataBegin = m_dataBegin + beginOffset;

            // assign rows and cols and stride according to inputs
            slice.m_rows = rto - rfrom;
            slice.m_rows = slice.m_rows % rstride ? slice.m_rows / rstride + 1 : slice.m_rows / rstride;

            slice.m_cols = cto - cfrom;
            slice.m_cols = slice.m_cols % cstride ? slice.m_cols / cstride + 1 : slice.m_cols / cstride;

            slice.m_size = slice.m_rows * slice.m_cols;
            slice.m_strides[0] = m_strides[0] * rstride;
            slice.m_strides[1] = m_strides[1] * cstride;

            return slice;
        }

        pointer m_data{};
        pointer m_dataBegin{};
        size_type m_rows{};
        size_type m_cols{};
        size_type m_size{};
        size_type m_strides[2]{};
        RefCount* m_refCounting{};
        MemoryResource* m_memoryResource{};

        /* This field indicates whether data contained resides in 3rd party image lib  or even from a raw pointer.
        We must not reference count these data.
        */
        bool m_ownsData{};
};

/*
 Equality operator. Check whether this object is equal to rhs according to implemented logic.

 @param[in] rhs object to check for equality.
 */
template<class T, class U, typename std::enable_if<AreCompatibleTypes<T, U>::value, int>::type = 0>
bool operator==(const Image<T>& lhs, const Image<U>& rhs) {
    if ((lhs.Rows() != rhs.Rows()) ||
        (lhs.Cols() != rhs.Cols()) || (lhs.Channels() != rhs.Channels())) {
        return false;
    }

    // reinterpret cast is needed for comparison primitive and pixel types i.e.
    // Image<std::uint8_t> == Image<Pixel<std::uint8_t,3>>
    return std::equal(lhs.begin(), lhs.end(), reinterpret_cast<typename Image<T>::const_iterator>(rhs.begin()));
}

/*
Equality operator. Check whether this object is equal to rhs according to implemented logic.

@param[in] rhs object to check for equality.

@return true if objects are equal, otherwise false.
*/
template<class T, class U, typename std::enable_if<!AreCompatibleTypes<T, U>::value, int>::type = 0>
bool operator==(const Image<T>& lhs, const Image<U>& rhs) {
    if ((lhs.GetType() != rhs.GetType()) || (lhs.Rows() != rhs.Rows()) ||
        (lhs.Cols() != rhs.Cols()) || (lhs.Channels() != rhs.Channels())) {
        return false;
    }

    if (lhs.Data() == reinterpret_cast<const T*>(rhs.Data())) {
        return true;
    }

    return std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

/*
Inequality operator. Check whether this object is not equal to rhs according to implemented logic.

@param[in] rhs object to check for inequality.

@return true if objects are not equal, otherwise false.
*/
template<class T, class U>
bool operator!=(const Image<T>& lhs, const Image<U>& rhs) {
    return !(lhs == rhs);
}

/**
    Convenience function to allocate newly transform image.

    @param[in] image Input image to be transformed.
    @param[in] f Unary functor using which

    @tparam T Output pixel type.
    @tparam U Input pixel type.
    @tparam UnaryFn Unary pure function, that transform U value from input image to the result / output buffer: f(U) -> T
*/
template<class T, class U, class UnaryFn>
Image<T> PixelTransform(const Image<U>& image, UnaryFn f) {
    static_assert(is_pixel_convertible<T, U>::value, "Provided image is not of convertible pixel type to this image.");
    CARBON_PRECONDITION(!image.IsEmpty(), "Provided image should not be empty.");

    Image<T> out(image.Rows(), image.Cols());
    out.PixelTransform(image, f);

    return out;
}

}  // namespace carbon
}  // namespace epic
