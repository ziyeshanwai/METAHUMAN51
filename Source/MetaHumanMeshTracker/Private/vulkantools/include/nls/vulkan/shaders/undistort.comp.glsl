// Copyright Epic Games, Inc. All Rights Reserved.

#version 450
#extension GL_ARB_separate_shader_objects : enable

#define WORKGROUP_SIZE 32
layout (local_size_x = WORKGROUP_SIZE, local_size_y = WORKGROUP_SIZE, local_size_z = 1 ) in;

/*
 * X = (x - cx) / fx
 * Y = (y - cy) / fy
 *
 * x = X/Z
 * y = Y/Z
 * r = sqrt(x^2 + y^2)
 * x' = x (1 + K1 r^2 + K2 r^4 + K3 r^6 + K4 r^8) + (P1 (r^2 + 2x^2) + 2 P2 x y) (1 + P3 r^2 + P4 r^4)
 * y' = y (1 + K1 r^2 + K2 r^4 + K3 r^6 + K4 r^8) + (P2 (r^2 + 2y^2) + 2 P1 x y) (1 + P3 r^2 + P4 r^4)
 * px = width * 0.5 + cx + x' f + x' B1 + y' B2
 * py = height * 0.5 + cy + y' f
 *
 * for texture lookup flip y coordinate of texture
 */
layout(binding = 0) uniform UniformBufferObject {
    int width;
    int height;
    float fx;
    float fy;
    float cx;
    float cy;
    float B1;
    float B2;
    vec4 radialDistortion;
    vec4 tangentialDistortion;
} ubo;

layout(binding = 1) uniform sampler2D distortedImageSampler;

layout(rgba32f, binding = 2) uniform writeonly image2D undistortedImageOut;


void main() {

  /*
  In order to fit the work into workgroups, some unnecessary threads are launched.
  We terminate those threads here.
  */
  if(gl_GlobalInvocationID.x >= ubo.width || gl_GlobalInvocationID.y >= ubo.height)
    return;

  vec2 uv = (vec2(gl_GlobalInvocationID.xy) + vec2(0.5, 0.5)) / vec2(ubo.width, ubo.height);
  uv.y = 1.0 - uv.y;

  const float X = (uv.x - ubo.cx) / ubo.fx;
  const float Y = (uv.y - ubo.cy) / ubo.fy;
  const float r2 = X * X + Y * Y;
  const float r4 = r2 * r2;
  const float r6 = r4 * r2;
  const float r8 = r4 * r4;
  const float K1 = ubo.radialDistortion[0];
  const float K2 = ubo.radialDistortion[1];
  const float K3 = ubo.radialDistortion[2];
  const float K4 = ubo.radialDistortion[3];
  const float P1 = ubo.tangentialDistortion[0];
  const float P2 = ubo.tangentialDistortion[1];
  const float P3 = ubo.tangentialDistortion[2];
  const float P4 = ubo.tangentialDistortion[3];
  const float radial = (1.0 + K1 * r2 + K2 * r4 + K3 * r6 + K4 * r8);
  const float tangentialX = P1 * (r2 + 2 * X * X) + 2 * P2 * X * Y;
  const float tangentialY = P2 * (r2 + 2 * Y * Y) + 2 * P1 * X * Y;
  const float xdash = X * radial + tangentialX * (1.0 + P3 * r2 + P4 * r4);
  const float ydash = Y * radial + tangentialY * (1.0 + P3 * r2 + P4 * r4);
  const float px = ubo.fx * xdash + ubo.cx + ubo.B1 * xdash / float(ubo.width) + ubo.B2 * ydash / float(ubo.width);
  const float py = ubo.fy * ydash + ubo.cy;

  vec4 color = texture(distortedImageSampler, vec2(px, py));
  const ivec2 coord = ivec2(gl_GlobalInvocationID.x, ubo.height - 1 - gl_GlobalInvocationID.y);
  imageStore(undistortedImageOut, coord, vec4(color.rgb, 1));
}
