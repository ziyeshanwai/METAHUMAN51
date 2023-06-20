// Copyright Epic Games, Inc. All Rights Reserved.

#version 450

// packed vertex data
struct Vertex {
    float x;
    float y;
    float z;
};

// packed texcoord data
struct Texcoord {
    float u;
    float v;
};

layout(push_constant) uniform UniformBufferObject {
    mat4 proj;
} ubo;

layout(std430, set = 1, binding = 0) readonly buffer VertexIndexBuffer {
    uint vertexIndices[];
};

layout(std430, set = 1, binding = 1) readonly buffer TexcoordIndexBuffer {
    uint texcoordIndices[];
};

layout(std430, set = 1, binding = 2) readonly buffer VertexBuffer {
    Vertex vertices[];
};

layout(std430, set = 1, binding = 3) readonly buffer TexcoordBuffer {
    vec2 texcoords[];
};

layout(location = 0) out vec4 fragTexCoord;

void main() {
    const uint vertexIndex = vertexIndices[gl_VertexIndex];
    const uint texcoordIndex = texcoordIndices[gl_VertexIndex];
    const Vertex vertex = vertices[vertexIndex];
    const vec2 texcoord = texcoords[texcoordIndex];

    gl_Position = ubo.proj * vec4(vertex.x, vertex.y, vertex.z, 1.0);
    fragTexCoord = vec4(texcoord.x, texcoord.y, 0.0, 1.0);
}
