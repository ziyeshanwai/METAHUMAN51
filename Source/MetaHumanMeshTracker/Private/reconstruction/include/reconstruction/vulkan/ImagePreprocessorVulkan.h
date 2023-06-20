// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/utils/Timecode.h>
#include <nls/geometry/ImageView.h>
#include <nls/geometry/Interpolation.h>
#include <nls/geometry/MetaShapeCamera.h>

#include <nls/vulkan/compute/VulkanDownsample.h>
#include <nls/vulkan/compute/VulkanExtractChannelFloat.h>
#include <nls/vulkan/compute/VulkanExtractChannelUint8.h>
#include <nls/vulkan/compute/VulkanExtractChannelUint16.h>
#include <nls/vulkan/compute/VulkanGaussianBlur.h>
#include <nls/vulkan/compute/VulkanHighPass.h>
#include <nls/vulkan/compute/VulkanWeightedAverage.h>
#include <nls/vulkan/compute/VulkanComputeUndistort.h>
#include <reconstruction/vulkan/VulkanCreateMask.h>

#include <map>
#include <memory>
#include <string>
#include <vector>


namespace epic::nls {

    /**
     * CameraImagePair contains a camera, an image, and a mask defining which pixels in the image are valid.
     */
    struct CameraImagePairVulkan {
        Camera<float> camera;
        std::shared_ptr<VulkanComputeBuffer> image;
        std::shared_ptr<VulkanComputeBuffer> mask;
        std::shared_ptr<VulkanComputeBuffer> highpass;
    };

    /**
     * CameraImagePairPyramid contains CameraImagePair for multiple resolution levels.
     */
    struct CameraImagePairPyramidVulkan {
        std::vector<CameraImagePairVulkan> imagePairs;
        carbon::Timecode timecode;

        int NumLevels() const {
            return int(imagePairs.size());
        }

        const epic::nls::Camera<float>& Camera(int level) const {
            return imagePairs[level].camera;
        }

        const std::shared_ptr<VulkanComputeBuffer>& Image(int level) const {
            return imagePairs[level].image;
        }

        const std::shared_ptr<VulkanComputeBuffer>& Mask(int level) const {
            return imagePairs[level].mask;
        }

        const std::shared_ptr<VulkanComputeBuffer>& Highpass(int level) const {
            return imagePairs[level].highpass;
        }

        // debug data
        std::shared_ptr<VulkanComputeBuffer> debugDistortedImage;
        std::shared_ptr<VulkanComputeBuffer> debugDistortedImageBlurred;
        std::shared_ptr<VulkanComputeBuffer> debugUndistortedImage;
        std::shared_ptr<VulkanComputeBuffer> debugUndistortedImageBlurred;
        std::shared_ptr<VulkanComputeBuffer> debugUndistortedMask;
    };


    class ImagePreprocessorVulkan {
        public:
            ImagePreprocessorVulkan(std::shared_ptr<VulkanComputeContext> computeContext)
                : m_computeContext(computeContext)
                , m_extractChannelFloat(computeContext)
                , m_extractChannelUint8(computeContext)
                , m_extractChannelUint16(computeContext)
                , m_downsample(computeContext)
                , m_gaussianBlur(std::make_shared<VulkanGaussianBlur>(computeContext))
                , m_undistort(computeContext)
                , m_createMask(computeContext)
                , m_weightedAverage(std::make_shared<VulkanWeightedAverage>(computeContext))
                , m_highPass(std::make_shared<VulkanHighPass>(m_gaussianBlur, m_weightedAverage))
            {}

            ImagePreprocessorVulkan(ImagePreprocessorVulkan&&) = default;

            typedef CameraImagePairPyramidVulkan Data;


            std::shared_ptr<Data> Process(const MetaShapeCamera<float>& camera,
                                          const ConstImageView& img,
                                          const std::vector<ConstImageView>& temporalKernelImgs,
                                          int numLevels);

            float InitialSigma() const { return m_initialSigma; }
            float DownsampleSigma() const { return m_downsampleSigma; }
            float DarkTreshold() const { return m_darkThreshold; }
            float BrightTreshold() const { return m_brightThreshold; }

            void SetInitialSigma(float sigma) { m_initialSigma = sigma; }
            void SetDownsampleSigma(float sigma) { m_downsampleSigma = sigma; }
            void SetDarkTreshold(float darkThreshold) { m_darkThreshold = darkThreshold; }
            void SetBrightTreshold(float brightThreshold) { m_brightThreshold = brightThreshold; }

            void EnableDebug(bool debug) { m_debug = debug; }

            void SetInterpolationMethod(epic::nls::InterpolationMethod interpolationMethod) { m_interpolationMethod = interpolationMethod; }

            epic::nls::InterpolationMethod InterpolationMethod() const { return m_interpolationMethod; }

            void SetSRGBInput(bool srgbInput) { m_srgbInput = srgbInput; }

            bool SRGBInput() const { return m_srgbInput;}

            Camera<float> OptimalUndistortedCameraForProcessing(const MetaShapeCamera<float>& camera, int numLevels) const;

        private:
            /**
             * @brief Converts an image to a single channel vulkan buffer. Extracts the green color channel for color images.
             *
             * @param img The image data. Supported formats are Monochrome, RGB, RGBA, in uint8, uint16, and float.
             * @param srgbInput Flag whether the image is srgb or linear space.
             * @param inputBuffer The input image already uploaded to the input buffer and with the required padding (4-byte aligned)
             * @param outBuffer The output buffer which must have size width * height
             */
            void ToVulkanBuffer(const ConstImageView& img, bool srgbInput, std::shared_ptr<VulkanComputeBuffer> inputBuffer, std::shared_ptr<VulkanComputeBuffer> outBuffer, VulkanComputeContext::HBatch batch);

        private:
            float m_initialSigma = 0.5f;
            float m_downsampleSigma = 0.75f;
            float m_darkThreshold = 0.03f;
            float m_brightThreshold = 0.99f;
            bool m_debug = false;
            // ! flag whether the input is in srgb color space and needs to be converted to linear space.
            bool m_srgbInput = false;
            epic::nls::InterpolationMethod m_interpolationMethod = epic::nls::InterpolationMethod::LINEAR;

        private:
            std::shared_ptr<VulkanComputeContext> m_computeContext;
            VulkanExtractChannelFloat m_extractChannelFloat;
            VulkanExtractChannelUint8 m_extractChannelUint8;
            VulkanExtractChannelUint16 m_extractChannelUint16;
            VulkanDownsample m_downsample;
            std::shared_ptr<VulkanGaussianBlur> m_gaussianBlur;
            VulkanComputeUndistort m_undistort;
            VulkanCreateMask m_createMask;
            std::shared_ptr<VulkanWeightedAverage> m_weightedAverage;
            std::shared_ptr<VulkanHighPass> m_highPass;
    };
}  // namespace epic::nls
