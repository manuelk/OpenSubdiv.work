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

#include "../common/glUtils.h"

static const char * g_meshShaderSrc =
#include "glMeshShader.gen.h"
;

static GLuint g_transformBinding=0,
              g_lightingBinding=1;

GLMesh::GLMesh(Topology const & topo, DrawMode drawMode) :
    _drawMode(drawMode) {

    // Shader
    _program = glCreateProgram();

    const std::string glsl_version = GLUtils::GetShaderVersionInclude();

    static char const vtxDefineStr[] = "#define VERTEX_SHADER\n",
                      geoDefineStr[] = "#define GEOMETRY_SHADER\n",
                      fragDefineStr[] = "#define FRAGMENT_SHADER\n";

    std::string vsSrc = glsl_version + vtxDefineStr + g_meshShaderSrc,
                fsSrc = glsl_version + fragDefineStr + g_meshShaderSrc;

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
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Locations
    GLint uboIndex = glGetUniformBlockIndex(_program, "Transform");
    if (uboIndex != GL_INVALID_INDEX)
        glUniformBlockBinding(_program, uboIndex, g_transformBinding);

    uboIndex = glGetUniformBlockIndex(_program, "Lighting");
    if (uboIndex != GL_INVALID_INDEX)
        glUniformBlockBinding(_program, uboIndex, g_lightingBinding);

    _attrPosition = glGetAttribLocation(_program, "my_position");
    _attrNormal = glGetAttribLocation(_program, "my_normal");
    _attrColor = glGetAttribLocation(_program, "my_color");

    glGenVertexArrays(1, &_vao);
    glBindVertexArray(_vao);

    //glEnableClientState( GL_VERTEX_ARRAY );
    //glEnableClientState( GL_NORMAL_ARRAY );

    std::vector<float> vertData(topo.nverts * 6);
    float * srcP = topo.positions,
          * srcN = topo.normals,
          * dst = &vertData[0];
    for (int i=0; i<topo.nverts; ++i, srcP+=3, srcN+=3) {
        memcpy(dst, srcP, 3 * sizeof(float)); dst += 3;
        memcpy(dst, srcN, 3 * sizeof(float)); dst += 3;
    }

    glGenBuffers(1, &_bufVertData);
    glBindBuffer(GL_ARRAY_BUFFER, _bufVertData);
    glBufferData(GL_ARRAY_BUFFER,vertData.size()*sizeof(GLfloat), &vertData[0], GL_STATIC_DRAW);

    glGenBuffers(1, &_bufColors);
    glBindBuffer(GL_ARRAY_BUFFER, _bufColors);
    glBufferData(GL_ARRAY_BUFFER, topo.nverts*3*sizeof(GLfloat), topo.colors, GL_STATIC_DRAW);

    _numVertices = topo.nverts;

    glBindVertexArray(0);

    GLUtils::CheckGLErrors("Create");
}

GLMesh::~GLMesh() {

    glDeleteProgram(_program);

    glDeleteVertexArrays(1, &_vao);

    glDeleteBuffers(1, &_bufVertData);
    glDeleteBuffers(1, &_bufColors);
}

int
GLMesh::GetNumTriangles() const {
    return _numVertices / 3;
}

void
GLMesh::Draw(GLuint xformUB, GLuint lightingUB, bool wireframe) const {

    glUseProgram(_program);

    glBindVertexArray(_vao);

    glBindBufferBase(GL_UNIFORM_BUFFER, g_transformBinding, xformUB);
    glBindBufferBase(GL_UNIFORM_BUFFER, g_lightingBinding, lightingUB);

    glBindBuffer(GL_ARRAY_BUFFER, _bufVertData);
    glEnableVertexAttribArray(_attrPosition);
    glVertexAttribPointer(_attrPosition, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), 0);
    glEnableVertexAttribArray(_attrNormal);
    glVertexAttribPointer(_attrNormal,   3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)(3 * sizeof(GLfloat)));

    glBindBuffer(GL_ARRAY_BUFFER, _bufColors);
    glEnableVertexAttribArray(_attrColor);
    glVertexAttribPointer(_attrColor, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);

    if (wireframe) {
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        glPointSize(2.0f);
        glLineWidth(1.0f);
        glEnable( GL_LINE_SMOOTH );
        if (_drawMode==DRAW_POINTS) {
            glDrawArrays(GL_POINTS, 0, _numVertices);
        } else if (_drawMode==DRAW_WIREFRAME) {
            glDrawArrays(GL_TRIANGLES, 0, _numVertices);
        }
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    } else {
        glDrawArrays(GL_TRIANGLES, 0, _numVertices);
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glUseProgram(0);
}

GLMesh::Topology
GLMesh::Topology::Cube() {

    static float positions[] = {
        -0.5f, -0.5f, -0.5f, /* A */ -0.5f, -0.5f,  0.5f, /* D */ -0.5f,  0.5f, -0.5f, /* B */
        -0.5f, -0.5f,  0.5f, /* D */ -0.5f,  0.5f,  0.5f, /* C */ -0.5f,  0.5f, -0.5f, /* B */

         0.5f, -0.5f,  0.5f, /* H */ -0.5f, -0.5f,  0.5f, /* D */ -0.5f, -0.5f, -0.5f, /* A */
         0.5f, -0.5f,  0.5f, /* H */ -0.5f, -0.5f, -0.5f, /* A */  0.5f, -0.5f, -0.5f, /* E */

         0.5f, -0.5f,  0.5f, /* H */  0.5f, -0.5f, -0.5f, /* E */  0.5f,  0.5f, -0.5f, /* F */
         0.5f, -0.5f,  0.5f, /* H */  0.5f,  0.5f, -0.5f, /* F */  0.5f,  0.5f,  0.5f, /* G */

         0.5f,  0.5f,  0.5f, /* G */  0.5f,  0.5f, -0.5f, /* F */ -0.5f,  0.5f, -0.5f, /* B */
         0.5f,  0.5f,  0.5f, /* G */ -0.5f,  0.5f, -0.5f, /* B */ -0.5f,  0.5f,  0.5f, /* C */

        -0.5f, -0.5f,  0.5f, /* D */  0.5f, -0.5f,  0.5f, /* H */  0.5f,  0.5f,  0.5f, /* G */
        -0.5f, -0.5f,  0.5f, /* D */  0.5f,  0.5f,  0.5f, /* G */  -0.5f,  0.5f,  0.5f, /* C */

        -0.5f, -0.5f, -0.5f, /* A */  0.5f,  0.5f, -0.5f, /* F */  0.5f, -0.5f, -0.5f, /* E */
        -0.5f, -0.5f, -0.5f, /* A */ -0.5f,  0.5f, -0.5f, /* B */  0.5f,  0.5f, -0.5f, /* F */
    };

    static float normals[] = {
        -1.0f,  0.0f,  0.0f,   -1.0f,  0.0f,  0.0f,   -1.0f,  0.0f,  0.0f,
        -1.0f,  0.0f,  0.0f,   -1.0f,  0.0f,  0.0f,   -1.0f,  0.0f,  0.0f,

         0.0f, -1.0f,  0.0f,    0.0f, -1.0f,  0.0f,    0.0f, -1.0f,  0.0f,
         0.0f, -1.0f,  0.0f,    0.0f, -1.0f,  0.0f,    0.0f, -1.0f,  0.0f,

         1.0f,  0.0f,  0.0f,    1.0f,  0.0f,  0.0f,    1.0f,  0.0f,  0.0f,
         1.0f,  0.0f,  0.0f,    1.0f,  0.0f,  0.0f,    1.0f,  0.0f,  0.0f,

         0.0f,  1.0f,  0.0f,    0.0f,  1.0f,  0.0f,    0.0f,  1.0f,  0.0f,
         0.0f,  1.0f,  0.0f,    0.0f,  1.0f,  0.0f,    0.0f,  1.0f,  0.0f,

         0.0f,  0.0f,  1.0f,    0.0f,  0.0f,  1.0f,    0.0f,  0.0f,  1.0f,
         0.0f,  0.0f,  1.0f,    0.0f,  0.0f,  1.0f,    0.0f,  0.0f,  1.0f,

         0.0f,  0.0f, -1.0f,    0.0f,  0.0f, -1.0f,    0.0f,  0.0f, -1.0f,
         0.0f,  0.0f, -1.0f,    0.0f,  0.0f, -1.0f,    0.0f,  0.0f, -1.0f,
    };

    static float colors[] = {
        1.0f, 0.5f, 0.5f,    1.0f, 0.5f, 0.5f,     1.0f, 0.5f, 0.5f,
        1.0f, 0.5f, 0.5f,    1.0f, 0.5f, 0.5f,     1.0f, 0.5f, 0.5f,

        0.5f, 1.0f, 0.5f,    0.5f, 1.0f, 0.5f,     0.5f, 1.0f, 0.5f,
        0.5f, 1.0f, 0.5f,    0.5f, 1.0f, 0.5f,     0.5f, 1.0f, 0.5f,

        0.5f, 0.5f, 1.0f,    0.5f, 0.5f, 1.0f,     0.5f, 0.5f, 1.0f,
        0.5f, 0.5f, 1.0f,    0.5f, 0.5f, 1.0f,     0.5f, 0.5f, 1.0f,

        1.0f, 1.0f, 0.5f,    1.0f, 1.0f, 0.5f,     1.0f, 1.0f, 0.5f,
        1.0f, 1.0f, 0.5f,    1.0f, 1.0f, 0.5f,     1.0f, 1.0f, 0.5f,

        0.5f, 1.0f, 1.0f,    0.5f, 1.0f, 1.0f,     0.5f, 1.0f, 1.0f,
        0.5f, 1.0f, 1.0f,    0.5f, 1.0f, 1.0f,     0.5f, 1.0f, 1.0f,

        1.0f, 0.5f, 1.0f,    1.0f, 0.5f, 1.0f,     1.0f, 0.5f, 1.0f,
        1.0f, 0.5f, 1.0f,    1.0f, 0.5f, 1.0f,     1.0f, 0.5f, 1.0f,
    };

    Topology topo;
    topo.positions = positions;
    topo.normals = normals;
    topo.colors = colors;
    topo.nverts = 36;

    return topo;
}

