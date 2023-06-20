// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/utils/Profiler.h>

#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/VulkanRectify.h>

#include <reconstruction/StereoCameraPair.h>
#include <reconstruction/vulkan/ImagePreprocessorVulkan.h>
#include <reconstruction/vulkan/VulkanStereoInitialMatching.h>
#include <reconstruction/vulkan/VulkanStereoUpsample.h>
#include <reconstruction/vulkan/VulkanStereoEstimateBrightnessScaling.h>
#include <reconstruction/vulkan/VulkanStereoMaskByBrightnessScaling.h>
#include <reconstruction/vulkan/VulkanStereoMaskConstraint.h>
#include <reconstruction/vulkan/VulkanStereoMatching.h>
#include <reconstruction/vulkan/VulkanStereoRematching.h>
#include <reconstruction/vulkan/VulkanStereoRefine.h>
#include <reconstruction/vulkan/VulkanStereoOrderingConstraint.h>
#include <reconstruction/vulkan/VulkanStereoSmoothnessConstraint.h>
#include <reconstruction/vulkan/VulkanStereoUniquenessConstraint.h>
#include <reconstruction/vulkan/VulkanDisparityToDepth.h>
#include <reconstruction/vulkan/VulkanDisparityGradientToMask.h>
#include <reconstruction/vulkan/VulkanDepthToDepthAndNormal.h>
#include <reconstruction/vulkan/VulkanDepthHallucinate.h>

namespace epic::nls {

/**
 * Holds the stereo reconstruction result.
 */
class StereoReconstructionResultVulkan
{
public:
    StereoReconstructionResultVulkan(std::shared_ptr<VulkanComputeContext> computeContext,
                                    std::shared_ptr<VulkanDisparityToDepth> disparityToDepth,
                                    std::shared_ptr<VulkanDepthToDepthAndNormal> depthToDepthAndNormal)
    : m_computeContext(computeContext)
    , m_disparityToDepth(disparityToDepth)
    , m_depthToDepthAndNormal(depthToDepthAndNormal)
    {}

  StereoCameraPair<float> stereoCameraPair;
  std::shared_ptr<VulkanComputeBuffer> disparitiesLeft;
  std::shared_ptr<VulkanComputeBuffer> disparitiesRight;
  std::shared_ptr<VulkanComputeBuffer> rectifiedImagesLeft;
  std::shared_ptr<VulkanComputeBuffer> rectifiedImagesRight;
  std::shared_ptr<VulkanComputeBuffer> maskImagesLeft;
  std::shared_ptr<VulkanComputeBuffer> maskImagesRight;
  std::shared_ptr<VulkanComputeBuffer> depthLeft;
  std::shared_ptr<VulkanComputeBuffer> depthRight;

  const StereoCameraPair<float>& StereoCameras() const { return stereoCameraPair; }

  std::shared_ptr<VulkanComputeBuffer> DepthAndNormalLeft();
  std::shared_ptr<VulkanComputeBuffer> DepthAndNormalRight();

private:
  std::shared_ptr<VulkanComputeBuffer> m_depthAndNormalLeft;
  std::shared_ptr<VulkanComputeBuffer> m_depthAndNormalRight;

  std::shared_ptr<VulkanComputeContext> m_computeContext;
  std::shared_ptr<VulkanDisparityToDepth> m_disparityToDepth;
  std::shared_ptr<VulkanDepthToDepthAndNormal> m_depthToDepthAndNormal;
};


/**
 * Class to do stereo reconstruction using Vulkan
 */
class StereoReconstructionVulkan
{
public:
  /**
   * Struct containing all of the result data.
   */
  struct Result {

    public:
      bool HasLevel(int level) const { return m_resultsPerLevel.find(level) != m_resultsPerLevel.end(); }
      const StereoReconstructionResultVulkan& Data(int level) const { return m_resultsPerLevel.find(level)->second; }
      StereoReconstructionResultVulkan& Data(int level) { return m_resultsPerLevel.find(level)->second; }
      void AddData(int level, StereoReconstructionResultVulkan&& result) { m_resultsPerLevel.emplace(level, std::move(result)); }
      void Clear() { m_resultsPerLevel.clear(); }

    private:
      std::map<int, StereoReconstructionResultVulkan> m_resultsPerLevel;
  };

public:
  StereoReconstructionVulkan(std::shared_ptr<VulkanComputeContext> computeContext)
    : m_computeContext(computeContext)
    , m_rectify(computeContext)
    , m_stereoInitialMatching(computeContext)
    , m_stereoUpsample(computeContext)
    , m_stereoEstimateBrightnessScaling(computeContext)
    , m_stereoMaskByBrightnessScaling(computeContext)
    , m_stereoMaskConstraint(computeContext)
    , m_stereoMatching(computeContext)
    , m_stereoRematching(computeContext)
    , m_stereoRefine(computeContext)
    , m_stereoOrderingConstraint(computeContext)
    , m_stereoSmoothnessConstraint(computeContext)
    , m_stereoUniquenessConstraint(computeContext)
    , m_disparityGradientToMask(computeContext)
    , m_disparityToDepth(std::make_shared<VulkanDisparityToDepth>(computeContext))
    , m_depthToDepthAndNormal(std::make_shared<VulkanDepthToDepthAndNormal>(computeContext))
    , m_depthHallucinate(std::make_shared<VulkanDepthHallucinate>(computeContext))
    , m_numLevels(0)
    , m_endLevel(0)
    , m_gpuTotalStats(GpuStats())
    , m_cpuTotalStats(CpuStats())
  {
  }


  Result Reconstruct(std::shared_ptr<const CameraImagePairPyramidVulkan> imagePyramidLeft,
                       std::shared_ptr<const CameraImagePairPyramidVulkan> imagePyramidRight,
                       const std::pair<float, float>& rangeLeft,
                       const std::pair<float, float>& rangeRight,
                       int endLevel = 0);

  void Reconstruct(Result& result, std::shared_ptr<const CameraImagePairPyramidVulkan> imagePyramidLeft,
                       std::shared_ptr<const CameraImagePairPyramidVulkan> imagePyramidRight,
                       const std::pair<float, float>& rangeLeft,
                       const std::pair<float, float>& rangeRight,
                       int endLevel = 0);

  float SmoothingWeight() const { return m_ws; }
  void SetSmoothingWeight(float ws) { m_ws = ws; }

  int SmoothingIterations() const { return m_iterations; }
  void SetSmoothingIterations(int iterations) { m_iterations = iterations; }

  int LastSmoothingIterations() const { return m_lastIterations; }
  void SetLastSmoothingIterations(int iterations) { m_lastIterations = iterations; }

  int NumMatchingOffsets() const { return m_numMatchingOffsets; }
  void SetNumMatchingOffsets(int numMatchingOffsets) { m_numMatchingOffsets = numMatchingOffsets; }

  int NumRematches() const { return m_numRematches; }
  void SetNumRematches(int numRematches) { m_numRematches = numRematches; }

  void SetInterpolationMethod(epic::nls::InterpolationMethod interpolationMethod) { m_interpolationMethod = interpolationMethod; }
  epic::nls::InterpolationMethod InterpolationMethod() const { return m_interpolationMethod; }

  //! Set a modification of the principal point for the right camera. This is purely for debugging purposes to see if a calibration change can improve the reconstruction.
  void SetPrincipalPointOffset(const Eigen::Vector2f& offset) { m_principalPointOffset = offset; }
  const Eigen::Vector2f& PrincipalPointOffset() const { return m_principalPointOffset; }

  float BrightnessScaleDifferenceThreshold() const { return m_scaleDifferenceThreshold; }
  void SetBrightnessScaleDifferenceThreshold(float threshold) { m_scaleDifferenceThreshold = threshold; }

  bool UseSegmentation() const { return m_useSegmentation; }
  void SetUseSegmentation(bool useSegmentation);

  float HallucinationScale() const { return m_hallucinationScale; }
  void SetHallucinationScale(float scale) { m_hallucinationScale = scale; }

  void PrintStats();

private:
  /**
   * Estimate the brightness scaling between the two images and mask the disparity map where it does not match.
   */
  void BrightnessScalingMask(
                        VulkanComputeContext::HBatch& batch,
                        std::shared_ptr<VulkanComputeBuffer> disparityMap,
                        std::shared_ptr<VulkanComputeBuffer> imgSrc,
                        std::shared_ptr<VulkanComputeBuffer> imgTarget,
                        std::shared_ptr<VulkanComputeBuffer> imgScaling,
                        const float scaleDifferenceThreshold);

private:
  static constexpr bool m_keepImageDimensionsInRectification = false;
  float m_ws = 0.005f;
  int m_iterations = 40;
  int m_lastIterations = 140;
  int m_numMatchingOffsets = 1;
  int m_numRematches = 5;
  epic::nls::InterpolationMethod m_interpolationMethod = epic::nls::InterpolationMethod::LINEAR;
  float m_scaleDifferenceThreshold = 0.2f;
  bool m_useSegmentation = false;
  float m_hallucinationScale = 0.0f;
  Eigen::Vector2f m_principalPointOffset = Eigen::Vector2f::Zero();

  std::shared_ptr<VulkanComputeContext> m_computeContext;
  VulkanRectify m_rectify;
  VulkanStereoInitialMatching m_stereoInitialMatching;
  VulkanStereoUpsample m_stereoUpsample;
  VulkanStereoEstimateBrightnessScaling m_stereoEstimateBrightnessScaling;
  VulkanStereoMaskByBrightnessScaling m_stereoMaskByBrightnessScaling;
  VulkanStereoMaskConstraint m_stereoMaskConstraint;
  VulkanStereoMatching m_stereoMatching;
  VulkanStereoRematching m_stereoRematching;
  VulkanStereoRefine m_stereoRefine;
  VulkanStereoOrderingConstraint m_stereoOrderingConstraint;
  VulkanStereoSmoothnessConstraint m_stereoSmoothnessConstraint;
  VulkanStereoUniquenessConstraint m_stereoUniquenessConstraint;
  VulkanDisparityGradientToMask m_disparityGradientToMask;
  std::shared_ptr<VulkanDisparityToDepth> m_disparityToDepth;
  std::shared_ptr<VulkanDepthToDepthAndNormal> m_depthToDepthAndNormal;
  std::shared_ptr<VulkanDepthHallucinate> m_depthHallucinate;

  struct CpuStats
  {
      double    timeInit;
      double    timeBufferAlloc;
      double    timeBrightnessScaling;
      double    timeBeforeWait;
      double    timeAfterWait;

      double    TotalTime() const
      {
          return
              timeInit +
              timeBufferAlloc +
              timeBrightnessScaling +
              timeBeforeWait +
              timeAfterWait;
      }
  };

  struct GpuStats
  {
      float   timeRectify{ 0.0f };
      float   timeRectifyMask{ 0.0f };
      float   timeInitialMatching{ 0.0f };
      float   timeEstimateBrightnessScaling{ 0.0f };
      float   timeBrightnessScalingMask{ 0.0f };
      float   timeUpsample{ 0.0f };
      float   timeMatching{ 0.0f };
      float   timeMaskConstraint{ 0.0f };
      float   timeSmoothnessConstraint{ 0.0f };
      float   timeOrderingConstraint{ 0.0f };
      float   timeUniquenessConstraint{ 0.0f };
      float   timeRematch{ 0.0f };
      float   timeRefine{ 0.0f };

      VulkanComputeContext::GpuTimeRange   rangeRectify;
      VulkanComputeContext::GpuTimeRange   rangeRectifyMask;
      VulkanComputeContext::GpuTimeRange   rangeInitialMatching;
      VulkanComputeContext::GpuTimeRange   rangeEstimateBrightnessScaling;
      VulkanComputeContext::GpuTimeRange   rangeBrightnessScalingMask;
      VulkanComputeContext::GpuTimeRange   rangeUpsample;
      VulkanComputeContext::GpuTimeRange   rangeMatching;
      VulkanComputeContext::GpuTimeRange   rangeMaskConstraint;
      VulkanComputeContext::GpuTimeRange   rangeSmoothnessConstraint;
      VulkanComputeContext::GpuTimeRange   rangeOrderingConstraint;
      VulkanComputeContext::GpuTimeRange   rangeUniquenessConstraint;
      VulkanComputeContext::GpuTimeRange   rangeRematch;
      VulkanComputeContext::GpuTimeRange   rangeRefine;

      float TotalTime() const
      {
          return
              timeRectify +
              timeRectifyMask +
              timeInitialMatching +
              timeEstimateBrightnessScaling +
              timeBrightnessScalingMask +
              timeUpsample +
              timeMaskConstraint +
              timeSmoothnessConstraint +
              timeOrderingConstraint +
              timeUniquenessConstraint +
              timeRematch +
              timeRefine;
      }
  };

  std::unique_ptr<std::pair<int,int>[]> m_levelSizes;
  int32_t                               m_numLevels;
  int32_t                               m_endLevel;
  std::unique_ptr<GpuStats[]>           m_gpuLevelStats;
  GpuStats                              m_gpuTotalStats;
  std::unique_ptr<CpuStats[]>           m_cpuLevelStats;
  CpuStats                              m_cpuTotalStats;
};



} // namespace epic::nls
