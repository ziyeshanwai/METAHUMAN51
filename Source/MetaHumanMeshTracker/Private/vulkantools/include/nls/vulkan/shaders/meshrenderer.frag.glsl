// Copyright Epic Games, Inc. All Rights Reserved.

#version 450
#extension GL_ARB_separate_shader_objects : enable

layout( push_constant ) uniform UniformBufferObject {
    mat4 proj;
    mat4 modelview;
    vec4 color;
    vec4 backColor;
    float shininess;
    float specIntensity;
    bool useLighting;
} ubo;

layout(location = 0) in vec3 inVertex;
layout(location = 1) in vec3 inNormal;

layout(location = 0) out vec4 outColor;

void main() {
    vec4 color = gl_FrontFacing ? ubo.color : ubo.backColor;
    if (ubo.useLighting) {
        const vec3 cameraDirection = normalize(inVertex);
        const vec3 normal = normalize(inNormal);
        const float cosAngle = dot(cameraDirection, normal);
        const float intensity = clamp(gl_FrontFacing ? -cosAngle : cosAngle, 0.0, 1.0);
        const vec3 diffuseColor = color.rgb * intensity;

        const float spec = pow(intensity, ubo.shininess);
        const vec3 specColor = vec3(ubo.specIntensity, ubo.specIntensity, ubo.specIntensity) * spec;
        outColor = vec4(diffuseColor + specColor, color.w);
    } else {
        outColor = color;
    }
}
