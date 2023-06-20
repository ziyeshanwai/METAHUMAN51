// Copyright Epic Games, Inc. All Rights Reserved.

#version 450
#extension GL_ARB_separate_shader_objects : enable

#define WORKGROUP_SIZE 32
layout (local_size_x = WORKGROUP_SIZE, local_size_y = 1, local_size_z = 1 ) in;

// packed vertex data
struct Vertex {
    float x;
    float y;
    float z;
};

layout(push_constant) uniform UniformBufferObject {
    mat4 texmodelviewproj;  // projection of vertex into texture
    mat4 texmodelview;      // transformation of vertex to texture space
    mat2 texprojinv;        // inverse projection from texture to vertices
    int numVertices;
} ubo;

layout(std430, set = 0, binding = 0) readonly buffer VertexBuffer {
    Vertex vertices[];
};

layout(std430, set = 0, binding = 1) readonly buffer NormalBuffer {
    Vertex normals[];
};

layout(std430, set = 0, binding = 2) writeonly buffer Visibility {
    float visibility[];
};

layout(set = 1, binding = 0) uniform sampler2D depthSampler;
layout(set = 2, binding = 0) uniform sampler2D depthAlphaSampler;


void main() {

  if(gl_GlobalInvocationID.x >= ubo.numVertices)
    return;

    const Vertex vertex = vertices[gl_GlobalInvocationID.x];
    const Vertex normal = normals[gl_GlobalInvocationID.x];

    const vec4 fragTexCoord = ubo.texmodelviewproj * vec4(vertex.x, vertex.y, vertex.z, 1.0);
    const vec4 vertexPositionInTextureSpace = ubo.texmodelview * vec4(vertex.x, vertex.y, vertex.z, 1.0);
    const vec3 cameraDirection = normalize(vertexPositionInTextureSpace.xyz);
    const vec3 normalPositionInTextureSpace = normalize((ubo.texmodelview * vec4(normal.x, normal.y, normal.z, 0.0)).xyz);

    const float cosAngle = max(0.0, -dot(cameraDirection, normalPositionInTextureSpace));

    const vec2 uv = fragTexCoord.xy / fragTexCoord.w;
    float invDepth = texture(depthSampler, uv).x;
    const float depthAlpha = texture(depthAlphaSampler, uv).w;
    if (depthAlpha > 0) {
        invDepth /= depthAlpha;
    }

    float totalVisibility = 0;

    if (invDepth > 0 && uv.x > 0 && uv.x < 1 && uv.y > 0 && uv.y < 1) {
        const vec2 renderedDepthPosition = ubo.texprojinv * vec2(invDepth, 1.0);
        const float renderedDepth = renderedDepthPosition.x / renderedDepthPosition.y;
        const float interpolatedDepth = vertexPositionInTextureSpace.z;
        const float depthVisibility = abs(renderedDepth - interpolatedDepth) < 0.1 ? 1.0 : 0.0;
        const float normalVisibility = cosAngle * cosAngle;
        totalVisibility = depthVisibility * normalVisibility;
    }

    visibility[gl_GlobalInvocationID.x] = totalVisibility;
}
