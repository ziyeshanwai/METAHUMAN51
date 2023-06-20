// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/io/ImageIO.h>

#include <filesystem>

#ifdef CARBON_USE_LIBTIFF
    #include <carbon/io/Tiff.h>
#endif

#ifdef CARBON_USE_OPENEXR
    #include <carbon/io/Exr.h>
#endif

#ifdef _MSC_VER
    #pragma warning(disable: 4702)
#endif

namespace epic::carbon {


namespace detail {

/*
    This class introduces signature for loading and saving all combination of image / pixel types.

    It is required to enable overloading different image handler types.
*/
struct AbstractImageHandler {

    // load

    virtual void Load(const std::string& filepath, Image<Mono8>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<MonoU16>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<MonoU32>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<MonoF>& image, ImageIOParameters* params) = 0;

    virtual void Load(const std::string& filepath, Image<MonoAlpha8>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<MonoAlphaU16>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<MonoAlphaU32>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<MonoAlphaF>& image, ImageIOParameters* params) = 0;

    virtual void Load(const std::string& filepath, Image<RGB8>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<RGBU16>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<RGBU32>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<RGBF>& image, ImageIOParameters* params) = 0;

    virtual void Load(const std::string& filepath, Image<RGBA8>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<RGBAU16>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<RGBAU32>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<RGBAF>& image, ImageIOParameters* params) = 0;

    virtual void Load(const std::string& filepath, Image<sRGB8>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<sRGBU16>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<sRGBU32>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<sRGBF>& image, ImageIOParameters* params) = 0;

    virtual void Load(const std::string& filepath, Image<sRGBA8>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<sRGBAU16>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<sRGBAU32>& image, ImageIOParameters* params) = 0;
    virtual void Load(const std::string& filepath, Image<sRGBAF>& image, ImageIOParameters* params) = 0;

    // save

    virtual void Save(const std::string& filepath, const Image<Mono8>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<MonoU16>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<MonoU32>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<MonoF>& image, const ImageIOParameters& params) = 0;

    virtual void Save(const std::string& filepath, const Image<MonoAlpha8>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<MonoAlphaU16>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<MonoAlphaU32>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<MonoAlphaF>& image, const ImageIOParameters& params) = 0;

    virtual void Save(const std::string& filepath, const Image<RGB8>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<RGBU16>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<RGBU32>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<RGBF>& image, const ImageIOParameters& params) = 0;

    virtual void Save(const std::string& filepath, const Image<RGBA8>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<RGBAU16>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<RGBAU32>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<RGBAF>& image, const ImageIOParameters& params) = 0;

    virtual void Save(const std::string& filepath, const Image<sRGB8>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<sRGBU16>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<sRGBU32>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<sRGBF>& image, const ImageIOParameters& params) = 0;

    virtual void Save(const std::string& filepath, const Image<sRGBA8>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<sRGBAU16>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<sRGBAU32>& image, const ImageIOParameters& params) = 0;
    virtual void Save(const std::string& filepath, const Image<sRGBAF>& image, const ImageIOParameters& params) = 0;
};


/*
    Image handling class. Specialized for each image type handler using it's meta-type.
*/
template<class HandlerType>
struct ImageHandler;

// All implemented handler meta-types (not necessarily included within this Carbon build)
struct Tiff;
struct Exr;


// Nifty loader / saver macros to minimize (as much as possible) overloading boilerplate

#define LOADER_METHOD(templateLoader, T) \
    void Load(const std::string& filepath, Image<T>& image, ImageIOParameters * params) override { \
        image = this->templateLoader<T>(filepath, params); \
    }

#define SAVER_METHOD(templateSaver, T) \
    void Save(const std::string& filepath, const Image<T>& image, const ImageIOParameters& params) override { \
        this->templateSaver<T>(filepath, image, params); \
    }

#define PACK_LOADER_OVERLOADS(templateLoader) \
    LOADER_METHOD(templateLoader, Mono8)   \
    LOADER_METHOD(templateLoader, MonoU16) \
    LOADER_METHOD(templateLoader, MonoU32) \
    LOADER_METHOD(templateLoader, MonoF) \
    LOADER_METHOD(templateLoader, MonoAlpha8) \
    LOADER_METHOD(templateLoader, MonoAlphaU16) \
    LOADER_METHOD(templateLoader, MonoAlphaU32) \
    LOADER_METHOD(templateLoader, MonoAlphaF) \
    LOADER_METHOD(templateLoader, RGB8) \
    LOADER_METHOD(templateLoader, RGBU16) \
    LOADER_METHOD(templateLoader, RGBU32) \
    LOADER_METHOD(templateLoader, RGBF) \
    LOADER_METHOD(templateLoader, RGBA8) \
    LOADER_METHOD(templateLoader, RGBAU16) \
    LOADER_METHOD(templateLoader, RGBAU32) \
    LOADER_METHOD(templateLoader, RGBAF) \
    LOADER_METHOD(templateLoader, sRGB8) \
    LOADER_METHOD(templateLoader, sRGBU16) \
    LOADER_METHOD(templateLoader, sRGBU32) \
    LOADER_METHOD(templateLoader, sRGBF) \
    LOADER_METHOD(templateLoader, sRGBA8) \
    LOADER_METHOD(templateLoader, sRGBAU16) \
    LOADER_METHOD(templateLoader, sRGBAU32) \
    LOADER_METHOD(templateLoader, sRGBAF)

#define PACK_SAVER_OVERLOADS(templateSaver) \
    SAVER_METHOD(templateSaver, Mono8)   \
    SAVER_METHOD(templateSaver, MonoU16) \
    SAVER_METHOD(templateSaver, MonoU32) \
    SAVER_METHOD(templateSaver, MonoF) \
    SAVER_METHOD(templateSaver, MonoAlpha8) \
    SAVER_METHOD(templateSaver, MonoAlphaU16) \
    SAVER_METHOD(templateSaver, MonoAlphaU32) \
    SAVER_METHOD(templateSaver, MonoAlphaF) \
    SAVER_METHOD(templateSaver, RGB8) \
    SAVER_METHOD(templateSaver, RGBU16) \
    SAVER_METHOD(templateSaver, RGBU32) \
    SAVER_METHOD(templateSaver, RGBF) \
    SAVER_METHOD(templateSaver, RGBA8) \
    SAVER_METHOD(templateSaver, RGBAU16) \
    SAVER_METHOD(templateSaver, RGBAU32) \
    SAVER_METHOD(templateSaver, RGBAF) \
    SAVER_METHOD(templateSaver, sRGB8) \
    SAVER_METHOD(templateSaver, sRGBU16) \
    SAVER_METHOD(templateSaver, sRGBU32) \
    SAVER_METHOD(templateSaver, sRGBF) \
    SAVER_METHOD(templateSaver, sRGBA8) \
    SAVER_METHOD(templateSaver, sRGBAU16) \
    SAVER_METHOD(templateSaver, sRGBAU32) \
    SAVER_METHOD(templateSaver, sRGBAF)


// General utility functions needed by all handlers
template<class T, class U, typename std::enable_if<std::is_same<T, U>::value, int>::type = 0>
Image<T> ReturnImage(Image<U>& image) {
    return std::move(image);
}

template<class T, class U,
         typename std::enable_if<!std::is_same<T, U>::value &&
                                 is_pixel_convertible<T, U>::value, int>::type = 0>
Image<T> ReturnImage(Image<U>&  image) {
    return image.template ConvertTo<T>();
}

template<class T, class U,
         typename std::enable_if<!std::is_same<T, U>::value &&
                                 !is_pixel_convertible<T, U>::value, int>::type = 0>
Image<T> ReturnImage(Image<U>&  /*image*/) {
    /*
        When Image<U> isn't convertible to Image<T>, this will throw. It is not
        really a favorable way to handle these things because the client has to
        know up front which types are convertible, and also, more importantly,
        they have to know what are they expecting of this image read op.

        Note that without this specialization, we'd get compile errors trying to
        run image.ConvertTo<T>() for U and T which are not convertible.
    */
    CARBON_CRITICAL("Failed converting read type to requested pixel type.");
}

#ifdef CARBON_USE_LIBTIFF
    template<>
    struct ImageHandler<Tiff> : public AbstractImageHandler {

        // overloading of loaders
        PACK_LOADER_OVERLOADS(Load_)

        // overloading of savers
        PACK_SAVER_OVERLOADS(Save_)

        private:
                /*
                    Explanation for these macros.

                    The trouble that I am trying to solve with macros
                    is hard-defined type for each image / pixel type,
                    or rather morphing between types, and supporting reading
                    for each of them. This type of type wrangling will
                    surely be present in other image handlers, therefore
                    it would be very desired that we abstract it outside.
                */
                #define read_image_typed(U) \
                    Image<U> image(props.height, props.width); \
                    ReadImage(filepath.c_str(), image.Data(), props); \
                    return ReturnImage<T>(image);

                #define read_image_typed_for_depth(U) \
                    switch (props.channels) { \
                        case 1: { \
                            read_image_typed(Mono<U>) \
                        } \
                        break; \
                        case 2: { \
                            read_image_typed(MonoAlpha<U>) \
                        } \
                        break; \
                        case 3: { \
                            read_image_typed(RGB<U>) \
                        } \
                        break; \
                        case 4: { \
                            read_image_typed(RGBA<U>) \
                        } \
                        break; \
                        default: \
                            CARBON_CRITICAL("Reading image of unsupported channel count."); \
                    }
                    
            template<class T>
            Image<T> Load_(const std::string&  filepath, ImageIOParameters*  /*params*/) {
                using namespace epic::carbon::tiff;

                ImageProperties props = ReadImageProperties(filepath.c_str());

                // For each possible sample format, bit depth and channel count, read image using proper Image<Pixel> type.
                Image<T> retImage;
                if (props.sampleFormat == SampleFormat::UINT) {
                    switch (props.bitDepth) {
                        case 1: {
                            read_image_typed_for_depth(std::uint8_t);
                        }
                        break;
                        case 2: {
                            read_image_typed_for_depth(std::uint16_t);
                        }
                        break;
                        case 4: {
                            read_image_typed_for_depth(std::uint32_t);
                        }
                        break;
                        default:
                            CARBON_CRITICAL("Bit depth of targeted image is unsupported.");
                    }
                } else if (props.sampleFormat == SampleFormat::IEEEFP) {
                    CARBON_ASSERT(props.bitDepth == 4,
                                  "Carbon TIFF reading support for 16bit floating point images is not yet implemented - only 32bit floats are supported.");
                    read_image_typed_for_depth(float);
                }
                    CARBON_CRITICAL("Read image is of non-supported format - only UINT and IEEEFP are supported.");
                
            }
            
              #undef read_image_typed
              #undef read_image_typed_for_depth


            tiff::Compression ParseCompression(const ImageIOParameters& params) {
                auto compressionIterator = params.find("compression");
                if (compressionIterator == params.end()) {
                    return tiff::Compression::NONE;
                }
                std::string compression = compressionIterator->second;

                if (compression == "none") {
                    return tiff::Compression::NONE;
                } else if (compression == "zip") {
                    return tiff::Compression::ADOBE_DEFLATE;
                } else if (compression == "lzw") {
                    return tiff::Compression::LZW;
                } else if (compression == "packbits") {
                    return tiff::Compression::PACKBITS;
                } else {
                    CARBON_CRITICAL("Provided compression setting ({}) is not supported. " \
                                    "Supported compression types for .tiff format: {none, zip, lzw, packbits}", compression);
                }
            }

            template<class T>
            void Save_(const std::string&  filepath, const Image<T>& image, const ImageIOParameters& params) {
                tiff::Compression compression = ParseCompression(params);

                tiff::WriteImage(filepath.c_str(),
                                 reinterpret_cast<const typename PixelTraits<T>::value_type*>(image.Data()),
                                 static_cast<std::uint32_t>(image.Cols()),
                                 static_cast<std::uint32_t>(image.Rows()),
                                 static_cast<std::uint32_t>(image.Channels()),
                                 compression);
            }

    };
#endif

#ifdef CARBON_USE_OPENEXR

    template<>
    struct ImageHandler<Exr> : public AbstractImageHandler {

        // overloading of loaders
        PACK_LOADER_OVERLOADS(load_)

        // overloading of savers
        PACK_SAVER_OVERLOADS(save_)

        private:
            exr::Compression ParseCompression(const ImageIOParameters& params) {
                auto compressionIterator = params.find("compression");
                if (compressionIterator == params.end()) {
                    return exr::Compression::NONE;
                }
                std::string compression = compressionIterator->second;

                if (compression == "none") {
                    return exr::Compression::NONE;
                } else if (compression == "rle") {
                    return exr::Compression::RLE;
                } else if (compression == "zips") {
                    return exr::Compression::ZIPS;
                } else if (compression == "zip") {
                    return exr::Compression::ZIP;
                } else if (compression == "piz") {
                    return exr::Compression::PIZ;
                } else if (compression == "pxr24") {
                    return exr::Compression::PXR24;
                } else if (compression == "b44") {
                    return exr::Compression::B44;
                } else if (compression == "b44a") {
                    return exr::Compression::B44A;
                } else if (compression == "dwaa") {
                    return exr::Compression::DWAA;
                } else if (compression == "dwab") {
                    return exr::Compression::DWAB;
                } else {
                    CARBON_CRITICAL("Provided compression setting ({}) is not supported. " \
                                    "Supported compression types for .exr format: {none, rle, zips, zip, piz, pxr24, b44, b44a, dwaa, dwab}",
                                    compression);
                }
            }


            #define convert_image_from(U) \
                   Image<U> result(reinterpret_cast<U*>(data), height, width, true);\
                   ::operator delete[](data);\
                   return ReturnImage<T>(result);
            
            #define convert_image_typed_for_depth(U) \
               switch (channels) { \
                   case exr::Channels::Y: { \
                       convert_image_from(Mono<U>) \
                   } \
                   break; \
                   case exr::Channels::YA: { \
                       convert_image_from(MonoAlpha<U>) \
                   } \
                   break; \
                   case exr::Channels::RGB: { \
                       convert_image_from(RGB<U>) \
                   } \
                   break; \
                   case exr::Channels::RGBA: { \
                       convert_image_from(RGBA<U>) \
                   } \
                   break; \
                   default: \
                       CARBON_CRITICAL("Reading image of unsupported channel count."); \
               }
 
            template<class T>
            Image<T> load_(const std::string&  filepath, ImageIOParameters*  /*params*/) {
                std::uint8_t* data{};
                int width{}, height{};

                try {
                    //Lets read whatever is in the picture and then try to convert it
                    exr::Channels channels = exr::Channels::ALL;
                    exr::PixelType pixelType{};
                    
                    exr::ReadImage(filepath.c_str(), &data, width, height, channels, pixelType);

                    if (pixelType == exr::PixelType::FLOAT)
                    {
                        convert_image_typed_for_depth(float);
                    }
                    else if (pixelType == exr::PixelType::UINT)
                    {
                        convert_image_typed_for_depth(std::uint32_t);
                    }
                    else
                    {
                        CARBON_CRITICAL("Unsupported pixel type: {} received when reding an .exr image at path \"{}\", error: {}", pixelType, filepath);
                    }
                    
                } catch (std::exception& e) {
                    CARBON_CRITICAL("Failed reading  .exr image at path \"{}\", error: {}", filepath, e.what());
                }
            }
        #undef convert_image_typed_for_depth
        #undef convert_image_from
           
            template<typename T>
            EPIC_CARBON_API struct EXR_WriteTypeValid
            {
            private:
                using type = typename PixelTraits<T>::value_type;
            public :
                static constexpr bool value = std::is_same<type, float>::value || std::is_same<type, std::uint32_t>::value;
            };

            template<class T,typename std::enable_if<EXR_WriteTypeValid<T>::value,int>::type = 0>
            void save_(const std::string&  filepath, const Image<T>&  image, const ImageIOParameters&  params) {

                exr::Compression compression = ParseCompression(params);
                exr::Channels channels = exr::EXR_Traits<T, typename PixelTraits<T>::value_type>::channels;
                try {

                    using write_type = typename exr::EXR_TypeTraits<typename PixelTraits<T>::value_type>::type;
                    exr::WriteImage<write_type>(filepath.c_str(),
                                    channels,
                                    reinterpret_cast<write_type*>(const_cast<T*>(image.Data())),
                                    static_cast<int>(image.Cols()),
                                    static_cast<int>(image.Rows()),
                                    compression);
                } catch (std::exception& e) {
                    CARBON_CRITICAL("Failed writing .exr image to path \"{}\", error: {}", filepath, e.what());
                }
            }

            template<class T, typename std::enable_if<!EXR_WriteTypeValid<T>::value, int>::type = 0>
            void save_(const std::string& /*filepath*/, const Image<T>& /*image*/, const ImageIOParameters& /*params*/) {
                CARBON_CRITICAL("Invalid pixel type. Only float and uint type suported. Conversion to one of these types is neccessary prior saving.");
            }
    };
#endif


// Image formats for existing (supported) image extensions.
const static std::map<std::string, AbstractImageHandler*> HANDLER_REGISTRY {
    #ifdef CARBON_USE_LIBTIFF
        {
            ".tif", new ImageHandler<Tiff>()
        },
        {
            // TODO: enable support for different extensions same loader (.tif|.tiff), (.jpg|.jpeg) etc
            ".tiff", new ImageHandler<Tiff>()
        },
    #endif

    #ifdef CARBON_USE_OPENEXR
        {
            ".exr", new ImageHandler<Exr>()
        }
    #endif
};

AbstractImageHandler* GetHandlerBasedOnExtension(const std::string& filepath) {
    std::string extension = std::filesystem::path(filepath).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](char c)->char {
                return static_cast<char>(std::tolower(c));
            });
    auto loader = detail::HANDLER_REGISTRY.find(extension);

    CARBON_POSTCONDITION(loader != detail::HANDLER_REGISTRY.end(),
                         "Carbon support for image format \"{}\" is not enabled.",
                         extension);

    return loader->second;
}

}

// Implementation of API read / write functions

template<class T>
Image<T> ReadImage(const std::string& filepath, ImageIOParameters* params) {
    auto handler = detail::GetHandlerBasedOnExtension(filepath);
    Image<T> image;
    handler->Load(filepath, image, params);

    return image;
}

template<class T>
void WriteImage(const std::string& filepath, const Image<T>& image, const ImageIOParameters& params) {
    auto handler = detail::GetHandlerBasedOnExtension(filepath);
    handler->Save(filepath, image, params);
}

// template instantiation of ReadImage and WriteImage

#if defined(CARBON_USE_LIBTIFF) || defined(CARBON_USE_OPENEXR)

    template Image<Mono8> ReadImage<Mono8>(const std::string& filepath, ImageIOParameters* params);
    template Image<MonoU16> ReadImage<MonoU16>(const std::string& filepath, ImageIOParameters* params);
    template Image<MonoU32> ReadImage<MonoU32>(const std::string& filepath, ImageIOParameters* params);
    template Image<MonoF> ReadImage<MonoF>(const std::string& filepath, ImageIOParameters* params);

    template Image<MonoAlpha8> ReadImage<MonoAlpha8>(const std::string& filepath, ImageIOParameters* params);
    template Image<MonoAlphaU16> ReadImage<MonoAlphaU16>(const std::string& filepath, ImageIOParameters* params);
    template Image<MonoAlphaU32> ReadImage<MonoAlphaU32>(const std::string& filepath, ImageIOParameters* params);
    template Image<MonoAlphaF> ReadImage<MonoAlphaF>(const std::string& filepath, ImageIOParameters* params);

    template Image<RGB8> ReadImage<RGB8>(const std::string& filepath, ImageIOParameters* params);
    template Image<RGBU16> ReadImage<RGBU16>(const std::string& filepath, ImageIOParameters* params);
    template Image<RGBU32> ReadImage<RGBU32>(const std::string& filepath, ImageIOParameters* params);
    template Image<RGBF> ReadImage<RGBF>(const std::string& filepath, ImageIOParameters* params);

    template Image<RGBA8> ReadImage<RGBA8>(const std::string& filepath, ImageIOParameters* params);
    template Image<RGBAU16> ReadImage<RGBAU16>(const std::string& filepath, ImageIOParameters* params);
    template Image<RGBAU32> ReadImage<RGBAU32>(const std::string& filepath, ImageIOParameters* params);
    template Image<RGBAF> ReadImage<RGBAF>(const std::string& filepath, ImageIOParameters* params);

    template Image<sRGB8> ReadImage<sRGB8>(const std::string& filepath, ImageIOParameters* params);
    template Image<sRGBU16> ReadImage<sRGBU16>(const std::string& filepath, ImageIOParameters* params);
    template Image<sRGBU32> ReadImage<sRGBU32>(const std::string& filepath, ImageIOParameters* params);
    template Image<sRGBF> ReadImage<sRGBF>(const std::string& filepath, ImageIOParameters* params);

    template Image<sRGBA8> ReadImage<sRGBA8>(const std::string& filepath, ImageIOParameters* params);
    template Image<sRGBAU16> ReadImage<sRGBAU16>(const std::string& filepath, ImageIOParameters* params);
    template Image<sRGBAU32> ReadImage<sRGBAU32>(const std::string& filepath, ImageIOParameters* params);
    template Image<sRGBAF> ReadImage<sRGBAF>(const std::string& filepath, ImageIOParameters* params);

    template void WriteImage<Mono8>(const std::string& filepath, const Image<Mono8>& image, const ImageIOParameters& params);
    template void WriteImage<MonoU16>(const std::string& filepath, const Image<MonoU16>& image, const ImageIOParameters& params);
    template void WriteImage<MonoU32>(const std::string& filepath, const Image<MonoU32>& image, const ImageIOParameters& params);
    template void WriteImage<MonoF>(const std::string& filepath, const Image<MonoF>& image, const ImageIOParameters& params);

    template void WriteImage<MonoAlpha8>(const std::string& filepath,const Image<MonoAlpha8>& image, const ImageIOParameters& params);
    template void WriteImage<MonoAlphaU16>(const std::string& filepath, const Image<MonoAlphaU16>& image, const ImageIOParameters& params);
    template void WriteImage<MonoAlphaU32>(const std::string& filepath, const Image<MonoAlphaU32>& image, const ImageIOParameters& params);
    template void WriteImage<MonoAlphaF>(const std::string& filepath, const Image<MonoAlphaF>& image, const ImageIOParameters& params);

    template void WriteImage<RGB8>(const std::string& filepath, const Image<RGB8>& image, const ImageIOParameters& params);
    template void WriteImage<RGBU16>(const std::string& filepath, const Image<RGBU16>& image, const ImageIOParameters& params);
    template void WriteImage<RGBU32>(const std::string& filepath, const Image<RGBU32>& image, const ImageIOParameters& params);
    template void WriteImage<RGBF>(const std::string& filepath, const Image<RGBF>& image, const ImageIOParameters& params);

    template void WriteImage<RGBA8>(const std::string& filepath, const Image<RGBA8>& image, const ImageIOParameters& params);
    template void WriteImage<RGBAU16>(const std::string& filepath, const Image<RGBAU16>& image, const ImageIOParameters& params);
    template void WriteImage<RGBAU32>(const std::string& filepath, const Image<RGBAU32>& image, const ImageIOParameters& params);
    template void WriteImage<RGBAF>(const std::string& filepath, const Image<RGBAF>& image, const ImageIOParameters& params);

    template void WriteImage<sRGB8>(const std::string& filepath, const Image<sRGB8>& image, const ImageIOParameters& params);
    template void WriteImage<sRGBU16>(const std::string& filepath, const Image<sRGBU16>& image, const ImageIOParameters& params);
    template void WriteImage<sRGBU32>(const std::string& filepath, const Image<sRGBU32>& image, const ImageIOParameters& params);
    template void WriteImage<sRGBF>(const std::string& filepath, const Image<sRGBF>& image, const ImageIOParameters& params);

    template void WriteImage<sRGBA8>(const std::string& filepath, const Image<sRGBA8>& image, const ImageIOParameters& params);
    template void WriteImage<sRGBAU16>(const std::string& filepath, const Image<sRGBAU16>& image, const ImageIOParameters& params);
    template void WriteImage<sRGBAU32>(const std::string& filepath, const Image<sRGBAU32>& image, const ImageIOParameters& params);
    template void WriteImage<sRGBAF>(const std::string& filepath, const Image<sRGBAF>& image, const ImageIOParameters& params);
#endif

}
