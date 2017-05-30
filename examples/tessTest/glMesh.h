//
//   Copyright 2016 Nvidia
//
//   Licensed under the Apache License, Version 2.0 (the "Apache License")
//   with the following modification; you may not use this file except in
//   compliance with the Apache License and the following modification to it:
//   Section 6. Trademarks. is deleted and replaced with:
//
//   6. Trademarks. This License does not grant permission to use the trade
//      names, trademarks, service marks, or product names of the Licensor
//      and its affiliates, except as required to comply with Section 4(c) of
//      the License and to reproduce the content of the NOTICE file.
//
//   You may obtain a copy of the Apache License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the Apache License with the above modification is
//   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
//   KIND, either express or implied. See the Apache License for the specific
//   language governing permissions and limitations under the Apache License.
//

#ifndef GL_MESH_H
#define GL_MESH_H

#include <osd/glVertexBuffer.h>

#include "../common/glUtils.h"

#include <vector>

// Mesh drawing
class GLMesh {

public:

    enum DrawMode {
        DRAW_POINTS,
        DRAW_WIREFRAME,
        DRAW_MESH
    };

    struct Topology {

        static Topology Cube();

        float * positions;
        float * normals;
        float * colors;
        int nverts;
    };

    GLMesh(Topology const & topo, DrawMode drawMode=DRAW_WIREFRAME);

    ~GLMesh();

    void Draw(GLuint xformUB, GLuint lightingUB, bool wireframe) const;

    int GetNumTriangles() const;

private:

    DrawMode _drawMode;

    int _numVertices;

    GLuint _program;

    GLint  _attrPosition,
           _attrNormal,
           _attrColor;

    GLuint _vao,
           _bufVertData,
           _bufColors;
};

#endif // GL_MESH_H
