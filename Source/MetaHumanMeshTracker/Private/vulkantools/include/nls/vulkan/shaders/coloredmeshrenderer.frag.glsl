// Copyright Epic Games, Inc. All Rights Reserved.

#version 450
#extension GL_ARB_separate_shader_objects : enable

layout( push_constant ) uniform UniformBufferObject {
    mat4 proj;
    mat4 modelview;
    bool useLighting;
} ubo;

layout(location = 0) in vec3 inVertex;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 inColor;

layout(location = 0) out vec4 outColor;

void main() {
    vec4 color = inColor;
    if (ubo.useLighting) {
        const vec3 cameraDirection = normalize(inVertex);
        const vec3 normal = normalize(inNormal);
        const float cosAngle = dot(cameraDirection, normal);
        float intensity = gl_FrontFacing ? -cosAngle : cosAngle;
        outColor = vec4(color.r * intensity, color.g * intensity, color.b * intensity, color.w);
    } else {
        outColor = color;
    }
}
