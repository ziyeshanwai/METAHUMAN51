// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <string>
#include <map>

#include <carbon/data/Image.h>


namespace epic {
namespace carbon {

/**
    String pairs offering possibility to receive and pass parameters from and to ReadImage and WriteImage.

    Each file handler offers its own parameter package. For now, only EXR and TIFF
    handlers are implemented, and both offer only compressions setting.

    For e.g. to name compression setting when writing an EXR image, you can define the parameter map
    value using initializer lists, as follows:

    @code
    WriteImage("path/to/image.exr", someImage, {{"compression": "pxr24"}});
    @endcode
*/
using ImageIOParameters = std::map<std::string, std::string>;

/**
    Read image from filesystem.

    Automatically infers image format from file extension.

    @warning So far only supported formats are TIFF and EXR, though both have to be enabled as
        Carbon extension (by default are excluded).

    @param [in] filepath Path where the image should be read from.
    @param [out] params Parameter storage of the read image. May contain metadata, color profile etc.

    @note Since the Image<T> type is templated, therefore strongly typed in compile time in terms
        of pixel value type, client has to define the output image type head of reading. This boils
        down to partially knowing the image type that is being read, because some pixel / color type
        conversion is not supported. If required pixel type is not convertible with true type of read
        image, std::runtime_error is thrown.

    @return Image that has been read, and potentially converted to the appropriate pixel type.
*/
template<class T>
Image<T> ReadImage(const std::string& filepath, ImageIOParameters* params = nullptr);


/**
    Write image to filesystem to a given file format, determined by the input path extension.

    @warning So far only supported formats are TIFF and EXR, though both have to be enabled as
        Carbon extension (by default are excluded).

    @param [in] filepath Path where the image should be written.
    @param [in] image Image for writing.
    @param [in] params Parameter package for written image. May contain metadata, color profile, compression methods etc.

    @note Only compression is supported at this point, and options depend on the image IO handler ie. format of image to
        be written.

    Tiff Parameters:
        - compression: none, zip, lzw, packbits

    Exr Parameters:
        - compression: none, rle, zips, zip, piz, pxr24, b44, b44a, dwaa, dwab
*/
template<class T>
void WriteImage(const std::string& filepath, const Image<T>& image, const ImageIOParameters& params = ImageIOParameters());


}  // carbon
}  // epic
