// Copyright Epic Games, Inc. All Rights Reserved.
#include <reconstruction/vulkan/StereoReconstructionVulkan.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>
#include <reconstruction/Rectification.h>

#include <carbon/Common.h>
#include <carbon/utils/Timer.h>

#include <algorithm>
#include <numeric>

#ifdef WITH_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace epic::nls {

    std::shared_ptr<VulkanComputeBuffer> StereoReconstructionResultVulkan::DepthAndNormalLeft() {
        if (m_depthAndNormalLeft) {
            return m_depthAndNormalLeft;
        }
        if (!depthLeft) {
            return std::shared_ptr<VulkanComputeBuffer>();
        }

        const int w = stereoCameraPair.CameraLeft().Width();
        const int h = stereoCameraPair.CameraLeft().Height();
        const Eigen::Matrix3f& K = stereoCameraPair.CameraLeft().Intrinsics();
        m_depthAndNormalLeft = VulkanComputeBuffer::Create(w * 4, h, m_computeContext);
        m_depthToDepthAndNormal->DepthToDepthAndNormal(depthLeft,
                                                       m_depthAndNormalLeft,
                                                       K(0, 0),
                                                       K(1, 1),
                                                       K(0, 1),
                                                       K(0, 2),
                                                       K(1, 2));
        return m_depthAndNormalLeft;
    }

    std::shared_ptr<VulkanComputeBuffer> StereoReconstructionResultVulkan::DepthAndNormalRight() {
        if (m_depthAndNormalRight) {
            return m_depthAndNormalRight;
        }
        if (!depthRight) {
            return std::shared_ptr<VulkanComputeBuffer>();
        }
        const int w = stereoCameraPair.CameraRight().Width();
        const int h = stereoCameraPair.CameraRight().Height();
        const Eigen::Matrix3f& K = stereoCameraPair.CameraRight().Intrinsics();
        m_depthAndNormalRight = VulkanComputeBuffer::Create(w * 4, h, m_computeContext);
        m_depthToDepthAndNormal->DepthToDepthAndNormal(depthRight, m_depthAndNormalRight, K(0, 0), K(1, 1), K(0, 1), K(0, 2),
                                                       K(1, 2));
        return m_depthAndNormalRight;
    }

    StereoReconstructionVulkan::Result StereoReconstructionVulkan::Reconstruct(
        std::shared_ptr<const CameraImagePairPyramidVulkan> imagePyramidL,
        std::shared_ptr<const CameraImagePairPyramidVulkan> imagePyramidR,
        const std::pair<float, float>& rangeL,
        const std::pair<float, float>& rangeR,
        int endLevel) {
        Result result;
        Reconstruct(result, imagePyramidL, imagePyramidR, rangeL, rangeR, endLevel);
        return result;
    }

    void StereoReconstructionVulkan::SetUseSegmentation(bool useSegmentation)
    {
#ifdef WITH_OPENCV
        m_useSegmentation = useSegmentation;
#else
        if (useSegmentation) {
            LOG_ERROR("no segmentation supported without OpenCV");
        }
#endif
    }

    void StereoReconstructionVulkan::Reconstruct(Result& result,
                     std::shared_ptr<const CameraImagePairPyramidVulkan> imagePyramidL,
                     std::shared_ptr<const CameraImagePairPyramidVulkan> imagePyramidR,
                     const std::pair<float, float>& rangeL,
                     const std::pair<float, float>& rangeR,
                     int endLevel) {
        PROFILING_FUNCTION(PROFILING_COLOR_PINK);

        // Timer totalTimer;
        Timer timer;

        const int numLevels = imagePyramidL->NumLevels();
        const int startLevel = numLevels - 1;
        const bool useBrightnessScalingMask = (m_scaleDifferenceThreshold > 0);

        Camera<float> newCameraL;
        Camera<float> newCameraR;
        std::shared_ptr<VulkanComputeBuffer> prevDisparityMapL;
        std::shared_ptr<VulkanComputeBuffer> prevDisparityMapR;

        m_levelSizes.reset(new std::pair<int, int>[numLevels]);
        m_numLevels = numLevels;
        m_endLevel = endLevel;
        m_gpuLevelStats.reset(new GpuStats[numLevels]);
        m_gpuTotalStats = {};
        m_cpuLevelStats.reset(new CpuStats[numLevels]);
        m_cpuTotalStats = {};

        // Store image level sizes and display them in PrintGpuTimes()
        for (int level = endLevel; level < numLevels; ++level)
        {
            auto image = imagePyramidL->Image(level);
            m_levelSizes[level].first = image->Width();
            m_levelSizes[level].second = image->Height();
        }

        for (int level = numLevels - 1; level >= endLevel; --level) {
            m_gpuLevelStats[level] = {};
            m_cpuLevelStats[level] = {};

            timer.Restart();
            m_computeContext->TimerReset();

            // LOG_INFO("vulkan level {} start time: {}", level, timer.Current()); timer.Restart();

            bool isStartLevel = (level == startLevel);

            const Camera<float>& cameraL = imagePyramidL->Camera(level);
            std::shared_ptr<VulkanComputeBuffer> imgL = imagePyramidL->Image(level);
            std::shared_ptr<VulkanComputeBuffer> maskL = imagePyramidL->Mask(level);
            Camera<float> cameraR = imagePyramidR->Camera(level);
            std::shared_ptr<VulkanComputeBuffer> imgR = imagePyramidR->Image(level);
            std::shared_ptr<VulkanComputeBuffer> maskR = imagePyramidR->Mask(level);

            if (m_principalPointOffset.norm() > 0) {
                auto intrinsics = cameraR.Intrinsics();
                intrinsics(0, 2) += m_principalPointOffset[0] / float(pow(2, level));
                intrinsics(1, 2) += m_principalPointOffset[1] / float(pow(2, level));
                cameraR.SetIntrinsics(intrinsics);
            }

            float maximumDisparity = 0;

            if (isStartLevel) {
                maximumDisparity = RectifyCameras(cameraL,
                                                  cameraR,
                                                  rangeL,
                                                  rangeR,
                                                  newCameraL,
                                                  newCameraR,
                                                  m_keepImageDimensionsInRectification);
                // LOG_INFO("maximum dispartiy: {}", maximumDisparity);
            } else {
                // scale camera from previous by level
                newCameraL.Scale(2.f);
                newCameraR.Scale(2.f);
            }

            if (result.HasLevel(level)) {
                // don't recompute if we already have the data of this level
                prevDisparityMapL = result.Data(level).disparitiesLeft;
                prevDisparityMapR = result.Data(level).disparitiesRight;
                newCameraL = result.Data(level).stereoCameraPair.CameraLeft();
                newCameraR = result.Data(level).stereoCameraPair.CameraRight();
                continue;
            }

            const StereoCameraPair<float> stereoCameraPair(newCameraL, newCameraR);

            m_cpuLevelStats[level].timeInit = timer.Current();
            timer.Restart();

            // LOG_INFO("vulkan setup time: {}", timer.Current()); timer.Restart();

            auto rectImgL = VulkanComputeBuffer::Create(newCameraL.Width(), newCameraL.Height(), m_computeContext);
            auto rectImgR = VulkanComputeBuffer::Create(newCameraR.Width(), newCameraR.Height(), m_computeContext);
            auto rectMaskL = VulkanComputeBuffer::Create(newCameraL.Width(), newCameraL.Height(), m_computeContext);
            auto rectMaskR = VulkanComputeBuffer::Create(newCameraR.Width(), newCameraR.Height(), m_computeContext);
            auto disparityMapL = VulkanComputeBuffer::Create(newCameraL.Width(), newCameraL.Height(), m_computeContext);
            auto disparityMapR = VulkanComputeBuffer::Create(newCameraR.Width(), newCameraR.Height(), m_computeContext);
            auto disparityMapLTmp = VulkanComputeBuffer::Create(newCameraL.Width(), newCameraL.Height(), m_computeContext);
            auto disparityMapRTmp = VulkanComputeBuffer::Create(newCameraR.Width(), newCameraR.Height(), m_computeContext);
            std::shared_ptr<VulkanComputeBuffer> scratchL;
            std::shared_ptr<VulkanComputeBuffer> scratchR;
            if (m_hallucinationScale > 0) {
                scratchL = VulkanComputeBuffer::Create(newCameraL.Width(), newCameraL.Height(), m_computeContext);
                scratchR = VulkanComputeBuffer::Create(newCameraR.Width(), newCameraR.Height(), m_computeContext);
            }

            std::shared_ptr<VulkanComputeBuffer> scalingImgL;
            std::shared_ptr<VulkanComputeBuffer> scalingImgR;

            if (isStartLevel && useBrightnessScalingMask)
            {
                scalingImgL = VulkanComputeBuffer::Create(disparityMapL->Width(), disparityMapL->Height(), m_computeContext);
                scalingImgR = VulkanComputeBuffer::Create(disparityMapR->Width(), disparityMapR->Height(), m_computeContext);
            }

            m_cpuLevelStats[level].timeBufferAlloc = timer.Current();
            timer.Restart();

            // LOG_INFO("vulkan alloc: {}", timer.Current()); timer.Restart();

            VulkanObjectName::Set(m_computeContext->Device()->Device(), rectImgL->ManagedBuffer()->Buffer(), "RectifyImage L");
            VulkanObjectName::Set(m_computeContext->Device()->Device(), rectImgR->ManagedBuffer()->Buffer(), "RectifyImage R");
            VulkanObjectName::Set(m_computeContext->Device()->Device(), rectMaskL->ManagedBuffer()->Buffer(), "RectifyMask L");
            VulkanObjectName::Set(m_computeContext->Device()->Device(), rectMaskR->ManagedBuffer()->Buffer(), "RectifyMask R");
            VulkanObjectName::Set(m_computeContext->Device()->Device(), disparityMapL->ManagedBuffer()->Buffer(), "DisparityMap L");
            VulkanObjectName::Set(m_computeContext->Device()->Device(), disparityMapR->ManagedBuffer()->Buffer(), "DisparityMap R");
            VulkanObjectName::Set(m_computeContext->Device()->Device(), disparityMapLTmp->ManagedBuffer()->Buffer(), "DisparityMap L Temp");
            VulkanObjectName::Set(m_computeContext->Device()->Device(), disparityMapRTmp->ManagedBuffer()->Buffer(), "DisparityMap R Temp");

            auto batchRectify = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeRectify);
                m_rectify.Rectify(imgL, rectImgL, cameraL, newCameraL, m_interpolationMethod, batchRectify);
                m_rectify.Rectify(imgR, rectImgR, cameraR, newCameraR, m_interpolationMethod, batchRectify);
            m_computeContext->BatchEnd(batchRectify);
            // LOG_INFO("vulkan rectify time: {}", timer.Current()); timer.Restart();

            auto batchRectifyMask = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeRectifyMask);
                m_rectify.Rectify(maskL, rectMaskL, cameraL, newCameraL, m_interpolationMethod, batchRectifyMask);
                m_rectify.Rectify(maskR, rectMaskR, cameraR, newCameraR, m_interpolationMethod, batchRectifyMask);
            m_computeContext->BatchEnd(batchRectifyMask);
            // LOG_INFO("vulkan rectify mask time: {}", timer.Current()); timer.Restart();

            VulkanComputeContext::HBatch    batchInitialMatching;
            VulkanComputeContext::HBatch    batchEstimateBrightnessScaling;
            VulkanComputeContext::HBatch    batchBrightnessScalingMask;
            VulkanComputeContext::HBatch    batchUpsample;
            VulkanComputeContext::HBatch    batchMatching;

            if (isStartLevel) {

                batchInitialMatching = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeInitialMatching);
                    m_stereoInitialMatching.InitialMatching(rectImgL, rectImgR, disparityMapL,
                                                            int(std::floor(-maximumDisparity)), 0,
                                                            batchInitialMatching);
                    m_stereoInitialMatching.InitialMatching(rectImgR, rectImgL, disparityMapR,
                                                            0, int(std::ceil(maximumDisparity)),
                                                            batchInitialMatching);
                m_computeContext->BatchEnd(batchInitialMatching);
                // LOG_INFO("vulkan initial matching time: {}", timer.Current()); timer.Restart();

                if (useBrightnessScalingMask) {
                    batchEstimateBrightnessScaling = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeEstimateBrightnessScaling);

                    m_stereoEstimateBrightnessScaling.EstimateBrightnessScaling(disparityMapL, rectImgL, rectImgR, scalingImgL, batchEstimateBrightnessScaling);
                    m_stereoEstimateBrightnessScaling.EstimateBrightnessScaling(disparityMapR, rectImgR, rectImgL, scalingImgR, batchEstimateBrightnessScaling);

                    m_computeContext->BatchEnd(batchEstimateBrightnessScaling);

                    batchBrightnessScalingMask = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeBrightnessScalingMask);

                    BrightnessScalingMask(batchBrightnessScalingMask, disparityMapL, rectImgL, rectImgR, scalingImgL, m_scaleDifferenceThreshold);
                    BrightnessScalingMask(batchBrightnessScalingMask, disparityMapR, rectImgR, rectImgL, scalingImgR, m_scaleDifferenceThreshold);

                    m_cpuLevelStats[level].timeBrightnessScaling = timer.Current();
                    timer.Restart();

                    m_computeContext->BatchEnd(batchBrightnessScalingMask);
                    // LOG_INFO("vulkan brightness scaling time: {}", timer.Current()); timer.Restart();
                }
            } else {

                // initialize the disparity map from the previous level
                batchUpsample = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeUpsample);
                    m_stereoUpsample.Upsample(prevDisparityMapL, disparityMapL, batchUpsample);
                    m_stereoUpsample.Upsample(prevDisparityMapR, disparityMapR, batchUpsample);
                m_computeContext->BatchEnd(batchUpsample);
                // LOG_INFO("vulkan upsampling time: {}", timer.Current()); timer.Restart();

                batchMatching = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeMatching);
                    m_stereoMatching.Matching(rectImgL, rectImgR, disparityMapL, m_numMatchingOffsets, batchMatching);
                    m_stereoMatching.Matching(rectImgR, rectImgL, disparityMapR, m_numMatchingOffsets, batchMatching);
                m_computeContext->BatchEnd(batchMatching);
                // LOG_INFO("vulkan matching time: {}", timer.Current()); timer.Restart();
            }

            auto batchMaskConstraint = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeMaskConstraint);
                m_stereoMaskConstraint.MaskConstraint(disparityMapL, rectMaskL, rectMaskR, batchMaskConstraint);
                m_stereoMaskConstraint.MaskConstraint(disparityMapR, rectMaskR,rectMaskL, batchMaskConstraint);
            m_computeContext->BatchEnd(batchMaskConstraint);
            // LOG_INFO("vulkan discard wrong matches: {}", timer.Current()); timer.Restart();

            auto batchSmoothnessConstraint = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeSmoothnessConstraint);
                m_stereoSmoothnessConstraint.ApplySmoothnessConstraint(disparityMapL, disparityMapLTmp, batchSmoothnessConstraint);
                m_stereoSmoothnessConstraint.ApplySmoothnessConstraint(disparityMapR, disparityMapRTmp, batchSmoothnessConstraint);
            m_computeContext->BatchEnd(batchSmoothnessConstraint);

            std::swap(disparityMapL, disparityMapLTmp);
            std::swap(disparityMapR, disparityMapRTmp);
            // LOG_INFO("vulkan smoothness time: {}", timer.Current()); timer.Restart();

            auto batchOrderingConstraint = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeOrderingConstraint);
                m_stereoOrderingConstraint.ApplyOrderingConstraint(disparityMapL, disparityMapLTmp, batchOrderingConstraint);
                m_stereoOrderingConstraint.ApplyOrderingConstraint(disparityMapR, disparityMapRTmp, batchOrderingConstraint);
            m_computeContext->BatchEnd(batchOrderingConstraint);

            std::swap(disparityMapL, disparityMapLTmp);
            std::swap(disparityMapR, disparityMapRTmp);
            // LOG_INFO("vulkan order time: {}", timer.Current()); timer.Restart();

            auto batchUniquenessConstraint = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeUniquenessConstraint);
                m_stereoUniquenessConstraint.ApplyUniquenessConstraint(disparityMapL, disparityMapR, disparityMapLTmp, batchUniquenessConstraint);
                m_stereoUniquenessConstraint.ApplyUniquenessConstraint(disparityMapR, disparityMapL, disparityMapRTmp, batchUniquenessConstraint);
            m_computeContext->BatchEnd(batchUniquenessConstraint);

            std::swap(disparityMapL, disparityMapLTmp);
            std::swap(disparityMapR, disparityMapRTmp);
            // LOG_INFO("vulkan uniqueness time: {}", timer.Current()); timer.Restart();

            auto batchRematch = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeRematch);

            for (int k = 0; k < m_numRematches; k++) {
                m_stereoRematching.Rematch(rectImgL, rectImgR, disparityMapL, disparityMapLTmp, batchRematch);
                m_stereoRematching.Rematch(rectImgR, rectImgL, disparityMapR, disparityMapRTmp, batchRematch);
                std::swap(disparityMapL, disparityMapLTmp);
                std::swap(disparityMapR, disparityMapRTmp);
            }

            m_computeContext->BatchEnd(batchRematch);
            // LOG_INFO("vulkan rematching time: {}", timer.Current()); timer.Restart();

            // // DO NOT USE: seems to create a lot of holes in the depth map
            // m_stereoUniquenessConstraint.ApplyUniquenessConstraint(disparityMapL, disparityMapR, disparityMapLTmp);
            // m_stereoUniquenessConstraint.ApplyUniquenessConstraint(disparityMapR, disparityMapL, disparityMapRTmp);
            // std::swap(disparityMapL, disparityMapLTmp);
            // std::swap(disparityMapR, disparityMapRTmp);

            auto batchRefine = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeRefine);

            for (int i = 0; i < ((level == 0) ? m_lastIterations : m_iterations); ++i) {
                m_stereoRefine.Refine(rectImgL, rectImgR, disparityMapL, disparityMapLTmp, m_ws, batchRefine);
                m_stereoRefine.Refine(rectImgR, rectImgL, disparityMapR, disparityMapRTmp, m_ws, batchRefine);
                std::swap(disparityMapL, disparityMapLTmp);
                std::swap(disparityMapR, disparityMapRTmp);
            }
            m_computeContext->BatchEnd(batchRefine);
            // LOG_INFO("vulkan refinement time: {}", timer.Current()); timer.Restart();

            auto batchApplyUniquenessConstraint2 = m_computeContext->BatchBegin();
                m_stereoUniquenessConstraint.ApplyUniquenessConstraint(disparityMapL, disparityMapR, disparityMapLTmp, batchApplyUniquenessConstraint2);
                m_stereoUniquenessConstraint.ApplyUniquenessConstraint(disparityMapR, disparityMapL, disparityMapRTmp, batchApplyUniquenessConstraint2);
            m_computeContext->BatchEnd(batchApplyUniquenessConstraint2);
            std::swap(disparityMapL, disparityMapLTmp);
            std::swap(disparityMapR, disparityMapRTmp);
            // LOG_INFO("vulkan uniqueness time: {}", timer.Current()); timer.Restart();

#ifdef WITH_OPENCV
            if (isStartLevel && m_useSegmentation) {
                m_computeContext->WaitForAll();

                // segmentation-based depth
                VulkanComputeContext::HBatch batch = m_computeContext->BatchBegin();
                const float disparityGradientThreshold = 5.0f;
                m_disparityGradientToMask.DisparityGradientToMask(disparityMapL, disparityMapLTmp, disparityGradientThreshold, batch);
                m_disparityGradientToMask.DisparityGradientToMask(disparityMapR, disparityMapRTmp, disparityGradientThreshold, batch);
                m_computeContext->BatchEnd(batch);
                m_computeContext->WaitForAll();

                cv::Mat1f dispValuesL(disparityMapLTmp->Height(), disparityMapLTmp->Width());
                cv::Mat1f dispValuesR(disparityMapRTmp->Height(), disparityMapRTmp->Width());
                disparityMapLTmp->CopyFromDevice(dispValuesL.data, disparityMapL->Height() * disparityMapL->Width() * sizeof(float));
                disparityMapRTmp->CopyFromDevice(dispValuesR.data, disparityMapR->Height() * disparityMapR->Width() * sizeof(float));

                cv::Mat1b imgMaskL(disparityMapL->Height(), disparityMapL->Width());
                cv::Mat1b imgMaskR(disparityMapR->Height(), disparityMapR->Width());
                for (int y = 0; y < imgMaskL.rows; ++y) {
                    for (int x = 0; x < imgMaskL.cols; ++x) {
                        imgMaskL(y,x) = (dispValuesL(y,x)) != 0 ? 255 : 0;
                    }
                }
                for (int y = 0; y < imgMaskR.rows; ++y) {
                    for (int x = 0; x < imgMaskR.cols; ++x) {
                        imgMaskR(y,x) = (dispValuesR(y,x)) != 0 ? 255 : 0;
                    }
                }

                cv::erode(imgMaskL, imgMaskL, cv::Mat(), cv::Point(-1,-1), 1);
                cv::erode(imgMaskR, imgMaskR, cv::Mat(), cv::Point(-1,-1), 1);

                auto keepBiggestComponent = [](cv::Mat1b& mask) {
                    cv::Mat labels;
                    cv::Mat stats;
                    cv::Mat centroids;
                    cv::connectedComponentsWithStats(mask, labels, stats, centroids);

                    std::vector<int> sortedLabels(stats.rows);
                    std::iota(sortedLabels.begin(), sortedLabels.end(), 1);
                    std::sort(sortedLabels.begin(), sortedLabels.end(), [&](int r1, int r2) { return stats.at<int>(r1, cv::CC_STAT_AREA) > stats.at<int>(r2, cv::CC_STAT_AREA); });
                    const int labelsToKeep = 2;

                    for (int y = 0; y < mask.rows; ++y) {
                        for (int x = 0; x < mask.cols; ++x) {
                            bool labelToKeep = false;
                            for (int k = 0; k < labelsToKeep; ++k) {
                                if (labels.at<int>(y, x) == sortedLabels[k]) labelToKeep = true;
                            }
                            mask(y,x) = labelToKeep ? 255 : 0;
                        }
                    }
                };
                keepBiggestComponent(imgMaskL);
                keepBiggestComponent(imgMaskR);

                cv::dilate(imgMaskL, imgMaskL, cv::Mat(), cv::Point(-1,-1), 1);
                cv::dilate(imgMaskR, imgMaskR, cv::Mat(), cv::Point(-1,-1), 1);

                for (int y = 0; y < imgMaskL.rows; ++y) {
                    for (int x = 0; x < imgMaskL.cols; ++x) {
                        if (imgMaskL(y,x) == 0) {
                            dispValuesL(y,x) = 0;
                        }
                    }
                }
                for (int y = 0; y < imgMaskR.rows; ++y) {
                    for (int x = 0; x < imgMaskR.cols; ++x) {
                        if (imgMaskR(y,x) == 0) {
                            dispValuesR(y,x) = 0;
                        }
                    }
                }

                disparityMapLTmp->CopyToDevice(dispValuesL.data, disparityMapL->Height() * disparityMapL->Width() * sizeof(float));
                disparityMapRTmp->CopyToDevice(dispValuesR.data, disparityMapR->Height() * disparityMapR->Width() * sizeof(float));

                auto batchSegmentationMaskConstraint = m_computeContext->BatchBegin(&m_gpuLevelStats[level].rangeMaskConstraint);
                    m_stereoMaskConstraint.MaskConstraint(disparityMapL, disparityMapLTmp, disparityMapRTmp, batchSegmentationMaskConstraint);
                    m_stereoMaskConstraint.MaskConstraint(disparityMapR, disparityMapRTmp, disparityMapLTmp, batchSegmentationMaskConstraint);
                m_computeContext->BatchEnd(batchSegmentationMaskConstraint);

                m_computeContext->WaitForAll();
            }
#endif

            auto batchApplyDisparityToDepth = m_computeContext->BatchBegin();
                m_disparityToDepth->DisparityToDepth(disparityMapL, disparityMapLTmp, stereoCameraPair.CameraLeft().Intrinsics()(0, 0), stereoCameraPair.Baseline(), stereoCameraPair.DisparityOffset(), -1.f, batchApplyDisparityToDepth);
                m_disparityToDepth->DisparityToDepth(disparityMapR, disparityMapRTmp, stereoCameraPair.CameraRight().Intrinsics()(0, 0), stereoCameraPair.Baseline(), stereoCameraPair.DisparityOffset(), 1.f, batchApplyDisparityToDepth);
            m_computeContext->BatchEnd(batchApplyDisparityToDepth);

            if (imagePyramidL->Highpass(level) && m_hallucinationScale > 0) {
                auto batchHallucinate = m_computeContext->BatchBegin();//&m_gpuLevelStats[level].rangeRectifyMask);
                    m_rectify.Rectify(imagePyramidL->Highpass(level), scratchL, cameraL, newCameraL, m_interpolationMethod, batchHallucinate);
                    m_depthHallucinate->DepthHallucinate(disparityMapLTmp, scratchL, disparityMapLTmp, m_hallucinationScale, batchHallucinate);
                m_computeContext->BatchEnd(batchHallucinate);
            }

            if (imagePyramidR->Highpass(level) && m_hallucinationScale > 0) {
                auto batchHallucinate = m_computeContext->BatchBegin();//&m_gpuLevelStats[level].rangeRectifyMask);
                    m_rectify.Rectify(imagePyramidR->Highpass(level), scratchR, cameraR, newCameraR, m_interpolationMethod, batchHallucinate);
                    m_depthHallucinate->DepthHallucinate(disparityMapRTmp, scratchR, disparityMapRTmp, m_hallucinationScale, batchHallucinate);
                m_computeContext->BatchEnd(batchHallucinate);
            }

            m_cpuLevelStats[level].timeBeforeWait = timer.Current();
            timer.Restart();

            m_computeContext->WaitForAll();
            m_cpuLevelStats[level].timeAfterWait = timer.Current();
            timer.Restart();

            // Get gpu kernel execution times for each batch
            m_gpuLevelStats[level].timeRectify     = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeRectify);
            m_gpuLevelStats[level].timeRectifyMask = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeRectifyMask);

            if (isStartLevel)
            {
                m_gpuLevelStats[level].timeInitialMatching = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeInitialMatching);
                m_gpuLevelStats[level].timeBrightnessScalingMask = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeBrightnessScalingMask);
            }
            else
            {
                m_gpuLevelStats[level].timeUpsample        = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeUpsample);
                m_gpuLevelStats[level].timeMaskConstraint  = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeMaskConstraint);
            }

            m_gpuLevelStats[level].timeSmoothnessConstraint    = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeSmoothnessConstraint);
            m_gpuLevelStats[level].timeOrderingConstraint      = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeOrderingConstraint);
            m_gpuLevelStats[level].timeUniquenessConstraint    = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeUniquenessConstraint);
            m_gpuLevelStats[level].timeRematch                 = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeRematch);
            m_gpuLevelStats[level].timeRefine                  = m_computeContext->TimerQueryValue(m_gpuLevelStats[level].rangeRefine);

            m_gpuTotalStats.timeRectify        += m_gpuLevelStats[level].timeRectify;
            m_gpuTotalStats.timeRectifyMask    += m_gpuLevelStats[level].timeRectifyMask;

            if (level == startLevel)
            {
                m_gpuTotalStats.timeInitialMatching        += m_gpuLevelStats[level].timeInitialMatching;
                m_gpuTotalStats.timeBrightnessScalingMask  += m_gpuLevelStats[level].timeBrightnessScalingMask;
            }
            else
            {
                m_gpuTotalStats.timeUpsample        += m_gpuLevelStats[level].timeUpsample;
                m_gpuTotalStats.timeMaskConstraint  += m_gpuLevelStats[level].timeMaskConstraint;
            }

            m_gpuTotalStats.timeSmoothnessConstraint    += m_gpuLevelStats[level].timeSmoothnessConstraint;
            m_gpuTotalStats.timeOrderingConstraint      += m_gpuLevelStats[level].timeOrderingConstraint;
            m_gpuTotalStats.timeUniquenessConstraint    += m_gpuLevelStats[level].timeUniquenessConstraint;
            m_gpuTotalStats.timeRematch                 += m_gpuLevelStats[level].timeRematch;
            m_gpuTotalStats.timeRefine                  += m_gpuLevelStats[level].timeRefine;

            m_cpuTotalStats.timeInit                += m_cpuLevelStats[level].timeInit;
            m_cpuTotalStats.timeBufferAlloc         += m_cpuLevelStats[level].timeBufferAlloc;
            m_cpuTotalStats.timeBrightnessScaling   += m_cpuLevelStats[level].timeBrightnessScaling;
            m_cpuTotalStats.timeBeforeWait          += m_cpuLevelStats[level].timeBeforeWait;
            m_cpuTotalStats.timeAfterWait           += m_cpuLevelStats[level].timeAfterWait;

            StereoReconstructionResultVulkan levelResult(m_computeContext, m_disparityToDepth, m_depthToDepthAndNormal);
            levelResult.stereoCameraPair = stereoCameraPair;
            levelResult.rectifiedImagesLeft = rectImgL;
            levelResult.rectifiedImagesRight = rectImgR;
            levelResult.maskImagesLeft = rectMaskL;
            levelResult.maskImagesRight = rectMaskR;
            levelResult.disparitiesLeft = disparityMapL;
            levelResult.disparitiesRight = disparityMapR;
            levelResult.depthLeft = disparityMapLTmp;
            levelResult.depthRight = disparityMapRTmp;
            result.AddData(level, std::move(levelResult));

            prevDisparityMapL = disparityMapL;
            prevDisparityMapR = disparityMapR;
        }
        // LOG_INFO("vulkan finalize time: {}", timer.Current()); timer.Restart();

        // LOG_INFO("vulkan total time: {}", totalTimer.Current()); totalTimer.Restart();

        // return result;
    }

    void StereoReconstructionVulkan::BrightnessScalingMask(
                                   VulkanComputeContext::HBatch&        batch,
                                   std::shared_ptr<VulkanComputeBuffer> disparityMap,
                                   std::shared_ptr<VulkanComputeBuffer> imgSrc,
                                   std::shared_ptr<VulkanComputeBuffer> imgTarget,
                                   std::shared_ptr<VulkanComputeBuffer> imgScaling,
                                   const float scaleDifferenceThreshold)
    {
        Eigen::VectorXf scalingValues(disparityMap->Width() * disparityMap->Height());
        imgScaling->CopyFromDevice(scalingValues.data(), scalingValues.size() * sizeof(float));

        int numBuckets = 128;
        Eigen::VectorXf scalingCount = Eigen::VectorXf::Zero(numBuckets + 1);
        const float minScale = 0.5f;
        const float maxScale = 2.0f;

        for (int i = 0; i < int(scalingValues.size()); i++) {
            const float scaling = scalingValues[i];
            if (scaling != 0) {
                const float offset =
                    std::max<float>(0, std::min<float>(1, (scaling - minScale) / (maxScale - minScale))) * (numBuckets -
                                                                                                            1);
                const int offsetIndex = int(std::floor(offset));
                const float w2 = offset - offsetIndex;
                scalingCount(offsetIndex + 0) += float(1) - w2;
                scalingCount(offsetIndex + 1) += w2;
            }
        }

        scalingCount[0] = 0;
        scalingCount[numBuckets - 1] = 0;
        scalingCount[numBuckets] = 0;
        typename Eigen::VectorXf::Index maxIndex;
        scalingCount.maxCoeff(&maxIndex);
        const float mainScale = float(maxIndex) / float(numBuckets - 1) * (maxScale - minScale) + minScale;

        // LOG_INFO("vulkan main scale: {}", mainScale);
        m_stereoMaskByBrightnessScaling.MaskByBrightnessScaling(disparityMap, imgSrc, imgTarget, mainScale, scaleDifferenceThreshold, batch);
    }

    void StereoReconstructionVulkan::PrintStats()
    {
        printf("Vulkan stereo reconstruction stats\n");

        printf("CPU total time\n");
        printf("   %-25s %.4fms\n", "init",                 m_cpuTotalStats.timeInit);
        printf("   %-25s %.4fms\n", "bufferAlloc",          m_cpuTotalStats.timeBufferAlloc);
        printf("   %-25s %.4fms\n", "brightnessScaling",    m_cpuTotalStats.timeBrightnessScaling);
        printf("   %-25s %.4fms\n", "beforeWait",           m_cpuTotalStats.timeBeforeWait);
        printf("   %-25s %.4fms\n", "afterWait",            m_cpuTotalStats.timeAfterWait);
        printf("   %-25s %.4fms\n", "totalCpuTime",         m_cpuTotalStats.TotalTime());

        printf("GPU total time\n");
        printf("   %-25s %.4fms\n", "rectify",              m_gpuTotalStats.timeRectify);
        printf("   %-25s %.4fms\n", "rectifyMask",          m_gpuTotalStats.timeRectifyMask);
        printf("   %-25s %.4fms\n", "initialMatching",      m_gpuTotalStats.timeInitialMatching);
        printf("   %-25s %.4fms\n", "estimateBrightness",   m_gpuTotalStats.timeEstimateBrightnessScaling);
        printf("   %-25s %.4fms\n", "brightnessMasking",    m_gpuTotalStats.timeBrightnessScalingMask);
        printf("   %-25s %.4fms\n", "upsample",             m_gpuTotalStats.timeUpsample);
        printf("   %-25s %.4fms\n", "maskConstraint",       m_gpuTotalStats.timeMaskConstraint);
        printf("   %-25s %.4fms\n", "smothnessConstraint",  m_gpuTotalStats.timeSmoothnessConstraint);
        printf("   %-25s %.4fms\n", "orderingConstraint",   m_gpuTotalStats.timeOrderingConstraint);
        printf("   %-25s %.4fms\n", "uniquenessConstraint", m_gpuTotalStats.timeUniquenessConstraint);
        printf("   %-25s %.4fms\n", "rematch",              m_gpuTotalStats.timeRematch);
        printf("   %-25s %.4fms\n", "refine",               m_gpuTotalStats.timeRefine);
        printf("   %-25s %.4fms\n", "totalGpuTime",         m_gpuTotalStats.TotalTime());

        for (int level = m_endLevel; level < m_numLevels; ++level)
        {
            printf("\n");
            printf("---------- level:%d (%d, %d)----------\n", level, m_levelSizes[level].first, m_levelSizes[level].second);

            printf("CPU\n");
            printf("   %-25s %.4fms\n", "init",                 m_cpuLevelStats[level].timeInit);
            printf("   %-25s %.4fms\n", "bufferAlloc",          m_cpuLevelStats[level].timeBufferAlloc);
            if (level == m_numLevels - 1)
                printf("   %-25s %.4fms\n", "brightnessScaling",    m_cpuLevelStats[level].timeBrightnessScaling);
            printf("   %-25s %.4fms\n", "beforeWait",           m_cpuLevelStats[level].timeBeforeWait);
            printf("   %-25s %.4fms\n", "afterWait",            m_cpuLevelStats[level].timeAfterWait);

            printf("GPU\n");
            printf("   %-25s %.4fms\n", "rectify",              m_gpuLevelStats[level].timeRectify);
            printf("   %-25s %.4fms\n", "rectifyMask",          m_gpuLevelStats[level].timeRectifyMask);
            if (level == m_numLevels - 1)
            {
                printf("   %-25s %.4fms\n", "initialMatching",      m_gpuLevelStats[level].timeInitialMatching);
                printf("   %-25s %.4fms\n", "estimateBrightness",   m_gpuLevelStats[level].timeEstimateBrightnessScaling);
                printf("   %-25s %.4fms\n", "brightnessMasking",    m_gpuLevelStats[level].timeBrightnessScalingMask);
            }
            else
            {
                printf("   %-25s %.4fms\n", "upsample",             m_gpuLevelStats[level].timeUpsample);
                printf("   %-25s %.4fms\n", "maskConstraint",       m_gpuLevelStats[level].timeMaskConstraint);
            }
            printf("   %-25s %.4fms\n", "smothnessConstraint",  m_gpuLevelStats[level].timeSmoothnessConstraint);
            printf("   %-25s %.4fms\n", "orderingConstraint",   m_gpuLevelStats[level].timeOrderingConstraint);
            printf("   %-25s %.4fms\n", "uniquenessConstraint", m_gpuLevelStats[level].timeUniquenessConstraint);
            printf("   %-25s %.4fms\n", "rematch",              m_gpuLevelStats[level].timeRematch);
            printf("   %-25s %.4fms\n", "refine",               m_gpuLevelStats[level].timeRefine);
            printf("   %-25s %.4fms\n", "totalGpuTime",         m_gpuLevelStats[level].TotalTime());
        }
    }
}  // namespace epic::nls
