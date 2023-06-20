// Copyright Epic Games, Inc. All Rights Reserved.

#version 450

layout( push_constant ) uniform UniformBufferObject {
    mat4 proj;
    mat4 modelview;
    vec4 color;
} ubo;

layout(location = 0) in vec3 inPosition;

layout(location = 0) out vec4 outColor;

void main() {

    gl_Position = ubo.proj * vec4(inPosition, 1.0);
    outColor = ubo.color;
}