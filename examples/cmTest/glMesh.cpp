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

#include "./glMesh.h"

#include <cassert>
#include <cstdlib>
#include <cstring>

static const char * g_meshShaderSrc =
#include "glMeshShader.gen.h"
;

GLuint g_uboIndex=GL_INVALID_INDEX,
       g_transformBinding=0,
       g_lightingBinding=1;


GLMesh::GLMesh(Topology const & topo) {

    // Shader
    _program = glCreateProgram(); 

    static char const versionStr[] = "#version 330\n",
                      vtxDefineStr[] = "#define VERTEX_SHADER\n",
                      geoDefineStr[] = "#define GEOMETRY_SHADER\n",
                      fragDefineStr[] = "#define FRAGMENT_SHADER\n";

    std::string vsSrc = std::string(versionStr) + vtxDefineStr + g_meshShaderSrc,
                fsSrc = std::string(versionStr) + fragDefineStr + g_meshShaderSrc;

    GLuint vertexShader = GLUtils::CompileShader(GL_VERTEX_SHADER, vsSrc.c_str()),
           fragmentShader = GLUtils::CompileShader(GL_FRAGMENT_SHADER, fsSrc.c_str());

    glAttachShader(_program, vertexShader);
    glAttachShader(_program, fragmentShader);
    glLinkProgram(_program);

    GLint status;
    glGetProgramiv(_program, GL_LINK_STATUS, &status);
    if (status == GL_FALSE) {
        GLint infoLogLength;
        glGetProgramiv(_program, GL_INFO_LOG_LENGTH, &infoLogLength);
        char *infoLog = new char[infoLogLength];
        glGetProgramInfoLog(_program, infoLogLength, NULL, infoLog);
        printf("%s\n", infoLog);
        delete[] infoLog;
        exit(1);
    }

    // Vertex / indices arrays
    glGenVertexArrays(1, &_vao);
    glBindVertexArray(_vao);

    glGenBuffers(1, &_ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        topo.nfaceVerts * sizeof(GLuint), topo.faceVerts, GL_STATIC_DRAW);

    _numIndices = topo.nfaceVerts;

    _vbo = GLVertexBuffer::Create(5, topo.nverts);

    glBindVertexArray(0);
}

GLMesh::~GLMesh() {

    glDeleteVertexArrays(1, &_vao);
    glDeleteBuffers(1, &_ebo);

    delete _vbo;
}

void GLMesh::Draw() const {

    glBindVertexArray(_vao);
    glDrawElements(GL_TRIANGLES, _numIndices, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}
