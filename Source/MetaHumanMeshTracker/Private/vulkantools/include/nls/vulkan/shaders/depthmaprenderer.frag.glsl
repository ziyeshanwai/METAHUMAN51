// Copyright Epic Games, Inc. All Rights Reserved.

#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec3 inVertex;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 inColor;

layout(location = 0) out vec4 outColor;

void main() {
    float value = 0.95 * -inNormal.z;
    outColor = vec4(value * inColor.x, value * inColor.y, value * inColor.z, inColor.w);
}
