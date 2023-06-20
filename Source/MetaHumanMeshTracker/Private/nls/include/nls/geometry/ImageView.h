// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/geometry/PixelFormat.h>

namespace epic::nls {

enum class PixelDepth
{
    UINT8,
    UINT16,
    FLOAT
};

inline int BytesForPixelDepth(PixelDepth pixelDepth)
{
    switch (pixelDepth) {
        case PixelDepth::UINT8: return 1;
        case PixelDepth::UINT16: return 2;
        case PixelDepth::FLOAT: return 4;
    }
    return 0;
}

class ImageView
{
public:
    ImageView(void* imageData, int width, int height, PixelFormat pixelFormat, PixelDepth pixelDepth)
        : m_imageData(imageData)
        , m_width(width)
        , m_height(height)
        , m_pixelFormat(pixelFormat)
        , m_pixelDepth(pixelDepth)
    {}

    void* Data() { return m_imageData; }
    const void* Data() const { return m_imageData; }
    int Width() const { return m_width; }
    int Height() const { return m_height; }
    PixelFormat GetPixelFormat() const { return m_pixelFormat;  }
    PixelDepth GetPixelDepth() const { return m_pixelDepth; }
    int GetSizeInBytes() const { return NumChannels(GetPixelFormat()) * BytesForPixelDepth(GetPixelDepth()) * Width() * Height(); }

private:
    void* m_imageData;
    int m_width;
    int m_height;
    PixelFormat m_pixelFormat;
    PixelDepth m_pixelDepth;
};

class ConstImageView
{
public:
    ConstImageView(const void* imageData, int width, int height, PixelFormat pixelFormat, PixelDepth pixelDepth)
        : m_imageData(imageData)
        , m_width(width)
        , m_height(height)
        , m_pixelFormat(pixelFormat)
        , m_pixelDepth(pixelDepth)
    {}

    ConstImageView(const ImageView& other)
        : m_imageData(other.Data())
        , m_width(other.Width())
        , m_height(other.Height())
        , m_pixelFormat(other.GetPixelFormat())
        , m_pixelDepth(other.GetPixelDepth())
    {
    }

    const void* Data() const { return m_imageData; }
    int Width() const { return m_width; }
    int Height() const { return m_height; }
    PixelFormat GetPixelFormat() const { return m_pixelFormat;  }
    PixelDepth GetPixelDepth() const { return m_pixelDepth; }
    int GetSizeInBytes() const { return NumChannels(GetPixelFormat()) * BytesForPixelDepth(GetPixelDepth()) * Width() * Height(); }

private:
    const void* m_imageData;
    int m_width;
    int m_height;
    PixelFormat m_pixelFormat;
    PixelDepth m_pixelDepth;
};


} //namespace epic::nls
