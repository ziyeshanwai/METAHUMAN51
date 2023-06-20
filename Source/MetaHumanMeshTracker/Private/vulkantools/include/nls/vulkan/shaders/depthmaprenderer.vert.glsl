// Copyright Epic Games, Inc. All Rights Reserved.

#version 450

layout( push_constant ) uniform UniformBufferObject {
    mat4 proj;
    mat4 modelview;
    vec4 color;
    uint depthMapWidth;
    float fx;
    float fy;
    float skew;
    float cx;
    float cy;
} ubo;

layout(location = 0) in vec4 depthAndNormal;

layout(location = 0) out vec3 outVertex;
layout(location = 1) out vec3 outNormal;
layout(location = 2) out vec4 outColor;

vec3 pixToPosition(float px, float py, float depth, float fx, float fy, float skew, float cx, float cy)
{
    const float y = (py - cy) / fy;
    const float x = (px - cx - y * skew) / fx;
    return vec3(x * depth, y * depth, depth);
}

void main() {
    const uint vertexIndex = gl_VertexIndex;

    gl_PointSize = 1;
    if (depthAndNormal.x > 0) {
        uint py = vertexIndex / ubo.depthMapWidth;
        uint px = vertexIndex - py * ubo.depthMapWidth;
        vec3 vertex = pixToPosition(px + 0.5, py + 0.5, depthAndNormal.x, ubo.fx, ubo.fy, ubo.skew, ubo.cx, ubo.cy);
        gl_Position = ubo.proj * vec4(vertex, 1.0);
        outVertex = (ubo.modelview * vec4(vertex, 1.0)).xyz;
        outNormal = (ubo.modelview * vec4(depthAndNormal.yzw, 0.0)).xyz;
    } else {
        gl_Position = vec4(0,0,0,1);
        outVertex = vec3(0,0,0);
        outNormal = vec3(0,0,0);
    }
    outColor = ubo.color;
}
