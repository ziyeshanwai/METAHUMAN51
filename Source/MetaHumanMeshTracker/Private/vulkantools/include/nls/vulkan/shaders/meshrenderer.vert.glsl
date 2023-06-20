// Copyright Epic Games, Inc. All Rights Reserved.

#version 450

layout( push_constant ) uniform UniformBufferObject {
    mat4 proj;
    mat4 modelview;
    vec4 color;
    vec4 backColor;
    float shininess;
    float specIntensity;
    bool useLighting;
} ubo;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

layout(location = 0) out vec3 outVertex;
layout(location = 1) out vec3 outNormal;

void main() {

    gl_Position = ubo.proj * vec4(inPosition, 1.0);
    outVertex = (ubo.modelview * vec4(inPosition, 1.0)).xyz;
    outNormal = (ubo.modelview * vec4(inNormal, 0.0)).xyz;
}
