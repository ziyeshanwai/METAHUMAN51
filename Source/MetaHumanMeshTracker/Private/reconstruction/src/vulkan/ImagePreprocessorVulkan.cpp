// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/ImagePreprocessorVulkan.h>

#include <carbon/Common.h>
#include <carbon/utils/Timer.h>
#include <pma/PolyAllocator.h>

using namespace pma;

namespace epic::nls {

Camera<float> ImagePreprocessorVulkan::OptimalUndistortedCameraForProcessing(const MetaShapeCamera<float>& camera, int numLevels) const
{
    const int padSize = int(std::pow(2, numLevels));

    const int width = camera.Width();
    const int height = camera.Height();
    const int undistortedWidth = static_cast<int>(camera.Undistort(Eigen::Vector2f(camera.Width(), float(camera.Height() / 2.0f)))[0] - camera.Undistort(Eigen::Vector2f(0, float(camera.Height() / 2.0f)))[0]);
    const int undistortedHeight = static_cast<int>(camera.Undistort(Eigen::Vector2f(float(camera.Width() / 2.0f), camera.Height()))[1] - camera.Undistort(Eigen::Vector2f(float(camera.Width() / 2.0f), 0))[1]);

    const int newWidth = int(std::ceil(undistortedWidth / float(padSize)) * padSize);
    const int newHeight = int(std::ceil(undistortedHeight / float(padSize)) * padSize);
    const int dcx = int((newWidth - width) * 0.5f);
    const int dcy = int((newHeight - height) * 0.5f);
    Eigen::Matrix<float, 3, 3> newIntrinsics = camera.Intrinsics();
    newIntrinsics(0, 2) += dcx;
    newIntrinsics(1, 2) += dcy;

    Camera<float> undistortedCamera;
    undistortedCamera.SetWidth(newWidth);
    undistortedCamera.SetHeight(newHeight);
    undistortedCamera.SetIntrinsics(newIntrinsics);
    undistortedCamera.SetExtrinsics(camera.Extrinsics());

    return undistortedCamera;
}

inline int PadToAlignment(int bytes, int alignment)
{
    const int offset = (bytes % alignment);
    if (offset != 0) {
        return bytes + alignment - offset;
    } else {
        return bytes;
    }
}

std::shared_ptr<ImagePreprocessorVulkan::Data> ImagePreprocessorVulkan::Process(const MetaShapeCamera<float>& camera,
                                                                                const ConstImageView& img,
                                                                                const std::vector<ConstImageView>& temporalKernelImgs,
                                                                                int numLevels)
{
    if (!m_computeContext) {
        CARBON_CRITICAL("missing compute context");
    }

    if (temporalKernelImgs.size() > 0) {
        LOG_WARNING("temporal noise reduction is not yet supported");
    }

    PolyAllocator<Data> polyAlloc{ MEM_RESOURCE };
    std::shared_ptr<Data> cameraImagePairPyramid = std::allocate_shared<CameraImagePairPyramidVulkan>(polyAlloc);

    const Camera<float> undistortedCamera = OptimalUndistortedCameraForProcessing(camera, numLevels);
    const int width = camera.Width();
    const int height = camera.Height();
    const int newWidth = undistortedCamera.Width();
    const int newHeight = undistortedCamera.Height();

    std::shared_ptr<VulkanComputeBuffer> distortedImage = VulkanComputeBuffer::Create(width, height, m_computeContext);
    std::shared_ptr<VulkanComputeBuffer> distortedImageBlurred;
    if (m_initialSigma > 0) {
        distortedImageBlurred = VulkanComputeBuffer::Create(width, height, m_computeContext);
    }
    std::shared_ptr<VulkanComputeBuffer> undistortedImage;
    std::shared_ptr<VulkanComputeBuffer> undistortedImageBlurred = VulkanComputeBuffer::Create(newWidth, newHeight, m_computeContext);
    std::shared_ptr<VulkanComputeBuffer> undistortedHighPassBlurred = VulkanComputeBuffer::Create(newWidth, newHeight, m_computeContext);
    // the scratch image is used to upload the raw image before extraction
    const int paddedInputImageSize = PadToAlignment(img.GetSizeInBytes(), sizeof(float)) / sizeof(float);
    const int scratchSize = std::max<int>(newWidth * newHeight, paddedInputImageSize);
    std::shared_ptr<VulkanComputeBuffer> scratchImage1 = VulkanComputeBuffer::Create(scratchSize, 1, m_computeContext);
    std::shared_ptr<VulkanComputeBuffer> scratchImage2 = VulkanComputeBuffer::Create(scratchSize, 1, m_computeContext);

    std::shared_ptr<VulkanComputeBuffer> distortedMask = VulkanComputeBuffer::Create(width, height, m_computeContext);
    std::shared_ptr<VulkanComputeBuffer> undistortedMask = VulkanComputeBuffer::Create(newWidth, newHeight, m_computeContext);

    if (m_debug) {
        undistortedImage = VulkanComputeBuffer::Create(newWidth, newHeight, m_computeContext);

        cameraImagePairPyramid->debugDistortedImage = distortedImage;
        cameraImagePairPyramid->debugDistortedImageBlurred = distortedImageBlurred;
        cameraImagePairPyramid->debugUndistortedImage = undistortedImage;
        cameraImagePairPyramid->debugUndistortedImageBlurred = undistortedImageBlurred;
        cameraImagePairPyramid->debugUndistortedMask = undistortedMask;
    }

    cameraImagePairPyramid->imagePairs.reserve(numLevels);
    cameraImagePairPyramid->imagePairs.push_back({undistortedCamera, undistortedImageBlurred, undistortedMask, undistortedHighPassBlurred});

    // allocate image pyramid data
    {
        Camera<float> currCamera = undistortedCamera;
        std::shared_ptr<VulkanComputeBuffer> curr = undistortedImageBlurred;
        for (int level = 1; level < numLevels; level++) {
            std::shared_ptr<VulkanComputeBuffer> downsampled = VulkanComputeBuffer::Create(curr->Width() / 2, curr->Height() / 2, m_computeContext);
            std::shared_ptr<VulkanComputeBuffer> downsampledMask = VulkanComputeBuffer::Create(curr->Width() / 2, curr->Height() / 2, m_computeContext);
            curr = downsampled;
            currCamera.Scale(0.5f);
            cameraImagePairPyramid->imagePairs.push_back({currCamera, downsampled, downsampledMask, nullptr});
        }
    }

    m_gaussianBlur->SetupGaussianKernel(m_initialSigma);
    m_gaussianBlur->SetupGaussianKernel(m_downsampleSigma);
    m_highPass->SetupKernels(0, 0, 3.0f, 15);

    scratchImage1->Resize(paddedInputImageSize, 1);
    scratchImage1->CopySubsetToDevice(img.Data(), 0, img.GetSizeInBytes());

    // if (temporalKernelImgs.size() > 0) {
    // distortedImage = OpticalFlowFiltering(img, temporalKernelImgs, m_srgbInput);
    // } else {
    VulkanComputeContext::HBatch batch = m_computeContext->BatchBegin();

    ToVulkanBuffer(img, m_srgbInput, scratchImage1, distortedImage, batch);

    // gaussian blur
    if (m_initialSigma > 0) {
        scratchImage1->Resize(distortedImage->Width(), distortedImage->Height());
        m_gaussianBlur->Blur(distortedImage, distortedImageBlurred, scratchImage1, m_initialSigma, batch);
    } else {
        distortedImageBlurred = distortedImage;
    }

    // undistort
    m_undistort.Undistort(distortedImageBlurred, undistortedImageBlurred, camera, undistortedCamera, m_interpolationMethod, batch);

    if (m_debug) {
        m_undistort.Undistort(distortedImage, undistortedImage, camera, undistortedCamera, m_interpolationMethod, batch);
    }

    // create mask and undistort
    m_createMask.CreateMask(distortedImage, distortedMask, batch, m_darkThreshold, m_brightThreshold);
    m_undistort.Undistort(distortedMask, undistortedMask, camera, undistortedCamera, m_interpolationMethod, batch);

    scratchImage1->Resize(undistortedImageBlurred->Width(), undistortedImageBlurred->Height());
    m_highPass->HighPass(undistortedImageBlurred, undistortedHighPassBlurred, scratchImage1, nullptr, /*scale=*/10.0f, /*delta=*/0.5f, batch);

    for (int level = 1; level < numLevels; level++) {
        std::shared_ptr<VulkanComputeBuffer> curr = cameraImagePairPyramid->imagePairs[level - 1].image;
        std::shared_ptr<VulkanComputeBuffer> currMask = cameraImagePairPyramid->imagePairs[level - 1].mask;
        std::shared_ptr<VulkanComputeBuffer> downsampled = cameraImagePairPyramid->imagePairs[level].image;
        std::shared_ptr<VulkanComputeBuffer> downsampledMask = cameraImagePairPyramid->imagePairs[level].mask;

        if (m_downsampleSigma > 0) {
            scratchImage1->Resize(curr->Width(), curr->Height());
            scratchImage2->Resize(curr->Width(), curr->Height());
            m_gaussianBlur->Blur(curr, scratchImage1, scratchImage2, m_downsampleSigma, batch);
            m_downsample.Downsample(scratchImage1, downsampled, batch);
        } else {
            m_downsample.Downsample(curr, downsampled, batch);
        }
        m_downsample.Downsample(currMask, downsampledMask, batch);
    }

    m_computeContext->BatchEnd(batch);
    m_computeContext->WaitForAll();

    return cameraImagePairPyramid;
}

void ImagePreprocessorVulkan::ToVulkanBuffer(const ConstImageView& img,
                                             bool srgbInput,
                                             std::shared_ptr<VulkanComputeBuffer> imgBuffer,
                                             std::shared_ptr<VulkanComputeBuffer> outBuffer,
                                             VulkanComputeContext::HBatch batch) {
    const int width = img.Width();
    const int height = img.Height();
    const int numChannels = NumChannels(img.GetPixelFormat());

    outBuffer->Resize(width, height);

    switch (img.GetPixelDepth()) {
        case PixelDepth::UINT8: {
            const int requiredByteSize = VulkanExtractChannelUint8::RequiredBufferByteSize(width, height, numChannels);
            if (imgBuffer->SizeInBytes() != requiredByteSize) {
                LOG_ERROR("image buffer does not have the required size in bytes for VulkanExtractChannelUint8: {} instead of {}", imgBuffer->SizeInBytes(), requiredByteSize);
            }
            switch (img.GetPixelFormat()) {
                case PixelFormat::MONOCHROME: {
                    m_extractChannelUint8.ExtractChannelUint8(imgBuffer, outBuffer, numChannels, 0, srgbInput, 1.0f / 255.f, batch);
                } break;
                case PixelFormat::BGR:
                case PixelFormat::RGB: {
                    m_extractChannelUint8.ExtractChannelUint8(imgBuffer, outBuffer, numChannels, 1, srgbInput, 1.0f / 255.f, batch);
                } break;
                case PixelFormat::BGRA:
                case PixelFormat::RGBA: {
                    m_extractChannelUint8.ExtractChannelUint8(imgBuffer, outBuffer, numChannels, 1, srgbInput, 1.0f / 255.f, batch);
                } break;
                default: CARBON_CRITICAL("unsupported image pixel format {}", img.GetPixelFormat());
            }
        } break;

        case PixelDepth::UINT16: {
            const int requiredByteSize = VulkanExtractChannelUint16::RequiredBufferByteSize(width, height, NumChannels(img.GetPixelFormat()));
            if (imgBuffer->SizeInBytes() != requiredByteSize) {
                LOG_ERROR("image buffer does not have the required size in bytes for VulkanExtractChannelUint16: {} instead of {}", imgBuffer->SizeInBytes(), requiredByteSize);
            }
            switch (img.GetPixelFormat()) {
                case PixelFormat::MONOCHROME: {
                    m_extractChannelUint16.ExtractChannelUint16(imgBuffer, outBuffer, numChannels, 0, srgbInput, 1.0f / 65535.0f, batch);
                } break;
                case PixelFormat::BGR:
                case PixelFormat::RGB: {
                    m_extractChannelUint16.ExtractChannelUint16(imgBuffer, outBuffer, numChannels, 1, srgbInput, 1.0f / 65535.0f, batch);
                } break;
                case PixelFormat::BGRA:
                case PixelFormat::RGBA: {
                    m_extractChannelUint16.ExtractChannelUint16(imgBuffer, outBuffer, numChannels, 1, srgbInput, 1.0f / 65535.0f, batch);
                } break;
                default: CARBON_CRITICAL("unsupported image pixel format {}", img.GetPixelFormat());
            }
        } break;

        case PixelDepth::FLOAT: {
            const int requiredByteSize = img.GetSizeInBytes();
            if (imgBuffer->SizeInBytes() != requiredByteSize) {
                LOG_ERROR("image buffer does not have the required size in bytes for VulkanExtractChannelFloat: {} instead of {}", imgBuffer->SizeInBytes(), requiredByteSize);
            }
            switch (img.GetPixelFormat()) {
                case PixelFormat::MONOCHROME: {
                    m_extractChannelFloat.ExtractChannelFloat(imgBuffer, outBuffer, numChannels, 0, srgbInput, 1.0f, batch);
                } break;
                case PixelFormat::BGR:
                case PixelFormat::RGB: {
                    m_extractChannelFloat.ExtractChannelFloat(imgBuffer, outBuffer, numChannels, 1, srgbInput, 1.0f, batch);
                } break;
                case PixelFormat::BGRA:
                case PixelFormat::RGBA: {
                    m_extractChannelFloat.ExtractChannelFloat(imgBuffer, outBuffer, numChannels, 1, srgbInput, 1.0f, batch);
                } break;
                default: CARBON_CRITICAL("unsupported image pixel format {}", img.GetPixelFormat());
            }
        } break;

        default: CARBON_CRITICAL("unsupported image pixel depth {}", img.GetPixelDepth());
    }
}

}  // namespace epic::nls
