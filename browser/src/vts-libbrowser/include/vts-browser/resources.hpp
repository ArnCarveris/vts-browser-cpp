/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RESOURCES_H_jhsegfshg
#define RESOURCES_H_jhsegfshg

#include <vector>
#include <memory>

#include "buffer.hpp"

namespace vts
{

// this is passed to the load* callbacks for the application to fill it in
class VTS_API ResourceInfo
{
public:
    ResourceInfo();

    // the userData is later on used to reference the corresponding resource
    //   from inside the DrawTask
    std::shared_ptr<void> userData;

    // memory usage in bytes
    uint32 ramMemoryCost;
    uint32 gpuMemoryCost;
};

// information about texture passed to loadTexture callback
// alternatively, it may be used to decode image file
class VTS_API GpuTextureSpec
{
public:
    GpuTextureSpec();
    GpuTextureSpec(const Buffer &buffer); // decode jpg or png file
    void verticalFlip();

    // image resolution
    uint32 width, height;

    // number of color channels per pixel
    uint32 components; // 1, 2, 3 or 4

    // raw texture data
    // it has (width * height * components) bytes
    // the rows are in no way aligned to multi-byte boundaries
    //   (GL_UNPACK_ALIGNMENT = 1)
    Buffer buffer;

    enum Flags
    {
        None = 0,
        Mipmaps = 1 << 0,
        Repeat = 1 << 1,
    } flags;
};

// information about mesh passed to loadMesh callback
// alternatively, it may be used to decode obj file
class VTS_API GpuMeshSpec
{
public:
    enum class FaceMode
    {
        // compatible with OpenGL
        Points = 0x0000,
        Lines = 0x0001,
        LineStrip = 0x0003,
        Triangles = 0x0004,
        TriangleStrip = 0x0005,
        TriangleFan = 0x0006,
    };

    struct VertexAttribute
    {
        enum class Type
        {
            // compatible with OpenGL
            Float = 0x1406,
            UnsignedByte = 0x1401,
        };

        VertexAttribute();
        uint32 offset; // in bytes
        uint32 stride; // in bytes
        uint32 components; // 1, 2, 3 or 4
        Type type;
        bool enable;
        bool normalized;
    };

    GpuMeshSpec();
    GpuMeshSpec(const Buffer &buffer); // decode obj file

    // an array of vertex data
    // the interpretation of the data is defined by the 'attributes' member
    Buffer vertices;

    // an array of uint16, or empty if the mesh is not indexed
    Buffer indices;

    // description of memory layout in the vertices buffer
    std::vector<VertexAttribute> attributes;

    uint32 verticesCount;
    uint32 indicesCount;
    FaceMode faceMode;
};

} // namespace vts

#endif
