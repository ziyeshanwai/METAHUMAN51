// Copyright Epic Games, Inc. All Rights Reserved.

#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform UniformBufferObject {
    mat4 uvtransform;       // transformation of uvs
    mat4 texmodelviewproj;  // projection of vertex into texture
    mat4 texmodelview;      // transformation of vertex to texture space
    mat2 texprojinv;        // inverse projection from texture to vertices
    bool forVisualization;  // in visualiztion mode we clamp the alpha value to properly render the color in the view
    bool redAndUV;          // only output the red channel (or gray for monochrome images), and output the projected camera position in the green and blue color channel
} ubo;

layout(binding = 0, set = 0) uniform sampler2D texSampler;
layout(binding = 0, set = 2) uniform sampler2D depthSampler;
layout(binding = 0, set = 3) uniform sampler2D depthAlphaSampler;

layout(location = 0) in vec4 fragTexCoord;
layout(location = 1) in vec4 vertexPositionInTextureSpace;
layout(location = 2) in vec4 normalPositionInTextureSpace;

layout(location = 0) out vec4 outColor;

void main() {
    const vec3 cameraDirection = normalize(vertexPositionInTextureSpace.xyz);
    const vec3 normal = normalize(normalPositionInTextureSpace.xyz);
    const float cosAngle = max(0.0, -dot(cameraDirection, normal));

    const vec2 uv = fragTexCoord.xy / fragTexCoord.w;
    float invDepth = texture(depthSampler, uv).x;
    const float depthAlpha = texture(depthAlphaSampler, uv).w;
    if (depthAlpha > 0) {
        invDepth /= depthAlpha;
    }
    const vec4 color = texture(texSampler, uv);
    outColor = color;
    if (ubo.redAndUV) {
        outColor.yz = textureSize(texSampler, 0) * uv;
    }

    if (invDepth > 0 && uv.x > 0 && uv.x < 1 && uv.y > 0 && uv.y < 1) {
        const vec2 renderedDepthPosition = ubo.texprojinv * vec2(invDepth, 1.0);
        const float renderedDepth = renderedDepthPosition.x / renderedDepthPosition.y;
        const float interpolatedDepth = vertexPositionInTextureSpace.z;
        const float depthVisibility = abs(renderedDepth - interpolatedDepth) < 0.1 ? 1.0 : 0.0;
        const float normalVisibility = cosAngle * cosAngle;
        const float totalVisibility = depthVisibility * normalVisibility;
        if (ubo.forVisualization) {
            if (totalVisibility > 0.1) {
                outColor.w = min(1.0, totalVisibility + 0.5);
            } else {
                outColor = vec4(0, 0, 0, 0);
            }
        } else {
            outColor.w = totalVisibility;
        }
    }
    else {
        if (ubo.forVisualization) {
            outColor = vec4(0, 0, 0, 0);
        } else {
            outColor.w = 0;
        }
    }
}
