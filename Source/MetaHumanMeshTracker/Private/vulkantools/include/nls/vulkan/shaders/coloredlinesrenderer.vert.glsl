// Copyright Epic Games, Inc. All Rights Reserved.

#version 450

layout( push_constant ) uniform UniformBufferObject {
    mat4 proj;
} ubo;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 outColor;

void main() {
    
    gl_Position = ubo.proj * vec4(inPosition, 1.0);
    outColor = inColor;
}
