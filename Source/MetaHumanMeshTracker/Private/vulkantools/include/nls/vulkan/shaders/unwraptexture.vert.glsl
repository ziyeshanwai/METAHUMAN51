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
    mat4 uvtransform;       // transformation of uvs
    mat4 texmodelviewproj;  // projection of vertex into texture
    mat4 texmodelview;      // transformation of vertex to texture space
    mat2 texprojinv;        // inverse projection from texture to vertices
    bool forVisualization;  // in visualiztion mode we clamp the alpha value to properly render the color in the view
    bool redAndUV;          // only output the red channel (or gray for monochrome images), and output the projected camera position in the green and blue color channel
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
    Texcoord texcoords[];
};

layout(std430, set = 1, binding = 4) readonly buffer NormalBuffer {
    Vertex normals[];
};

layout(location = 0) out vec4 fragTexCoord;
layout(location = 1) out vec4 vertexPositionInTextureSpace;
layout(location = 2) out vec4 normalPositionInTextureSpace;

void main() {
    const uint vertexIndex = vertexIndices[gl_VertexIndex];
    const uint texcoordIndex = texcoordIndices[gl_VertexIndex];
    const Vertex vertex = vertices[vertexIndex];
    const Texcoord texcoord = texcoords[texcoordIndex];
    const Vertex normal = normals[vertexIndex];

    gl_Position = ubo.uvtransform * vec4(texcoord.u, texcoord.v, 0.0, 1.0);
    fragTexCoord = ubo.texmodelviewproj * vec4(vertex.x, vertex.y, vertex.z, 1.0);
    vertexPositionInTextureSpace = ubo.texmodelview * vec4(vertex.x, vertex.y, vertex.z, 1.0);
    normalPositionInTextureSpace = ubo.texmodelview * vec4(normal.x, normal.y, normal.z, 0.0);
}
