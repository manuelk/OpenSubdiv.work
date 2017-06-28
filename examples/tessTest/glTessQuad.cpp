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


#include "./glTessQuad.h"

static const char * g_meshShaderSrc =
#include "glTessQuad.gen.h"
;

const int indices[] = { 0, 1, 2, 3 };

const float positions[] = { -1.0f, 0.0f, -1.0f,
                             1.0f, 0.0f, -1.0f,
                             1.0f, 0.0f,  1.0f,
                            -1.0f, 0.0f,  1.0f };

static GLuint g_transformBinding=0,
              g_lightingBinding=1;

GLTessQuad::GLTessQuad() {

    _program = 0;

    _spacingMode = FRACTIONAL_EVEN;

    _tessFactorInner[0] = _tessFactorInner[1] = 2.5f;

    _tessFactorOuter[0] = _tessFactorOuter[1] =
    _tessFactorOuter[2] = _tessFactorOuter[3] = 2.5f;
}

GLTessQuad::~GLTessQuad() {

    glDeleteProgram(_program);

    glDeleteVertexArrays(1, &_vao);

    glDeleteBuffers(1, &_indBuf);
    glDeleteBuffers(1, &_attrBuf);
}

void
GLTessQuad::rebuildProgram() {

    // Shader
    glDeleteProgram(_program);

    _program = glCreateProgram();

    const std::string glsl_version = GLUtils::GetShaderVersionInclude();

    static char const vsDefineStr[]  = "#define VERTEX_SHADER\n",
                      tcsDefineStr[] = "#define TESS_CONTROL_SHADER\n",
                      tesDefineStr[] = "#define TESS_EVAL_SHADER\n",
                      gsDefineStr[]  = "#define GEOMETRY_SHADER\n",
                      psDefineStr[]  = "#define FRAGMENT_SHADER\n";

    static char const * spacingNames[3] = { "fractional_odd_spacing", "fractional_even_spacing", "equal_spacing" };

    std::string spacingStr = std::string("#define SPACING_MODE ") + spacingNames[_spacingMode] + "\n";

    std::string vsSrc  = glsl_version + vsDefineStr  + g_meshShaderSrc,
                tcsSrc = glsl_version + tcsDefineStr + g_meshShaderSrc,
                tesSrc = glsl_version + spacingStr + tesDefineStr + g_meshShaderSrc,
                gsSrc  = glsl_version + gsDefineStr  + g_meshShaderSrc,
                psSrc  = glsl_version + psDefineStr  + g_meshShaderSrc;


    GLuint vsShader  = GLUtils::CompileShader(GL_VERTEX_SHADER, vsSrc.c_str()),
           tcsShader = GLUtils::CompileShader(GL_TESS_CONTROL_SHADER, tcsSrc.c_str()),
           tesShader = GLUtils::CompileShader(GL_TESS_EVALUATION_SHADER, tesSrc.c_str()),
           gsShader  = GLUtils::CompileShader(GL_GEOMETRY_SHADER, gsSrc.c_str()),
           psShader  = GLUtils::CompileShader(GL_FRAGMENT_SHADER, psSrc.c_str());

    glAttachShader(_program, vsShader);
    glAttachShader(_program, tcsShader);
    glAttachShader(_program, tesShader);
    glAttachShader(_program, gsShader);
    glAttachShader(_program, psShader);
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

    glDeleteShader(vsShader);
    glDeleteShader(tcsShader);
    glDeleteShader(tesShader);
    glDeleteShader(gsShader);
    glDeleteShader(psShader);
}

void
GLTessQuad::Init(float xofs, float yofs, float zofs) {

    glGenVertexArrays(1, &_vao);
    glBindVertexArray(_vao);

    float ofsPos[sizeof(positions)];
    for (int i=0; i<sizeof(positions)/3; ++i) {
        ofsPos[i*3+0] = positions[i*3+0] + xofs;
        ofsPos[i*3+1] = positions[i*3+1] + yofs;
        ofsPos[i*3+2] = positions[i*3+2] + zofs;
    }

    GLsizei stride = 3 * sizeof(float);
    glGenBuffers(1, &_attrBuf);
    glBindBuffer(GL_ARRAY_BUFFER, _attrBuf);
    glBufferData(GL_ARRAY_BUFFER, sizeof(positions), ofsPos, GL_STATIC_DRAW);

    _posAttr = 0;
    glEnableVertexAttribArray(_posAttr);
    glVertexAttribPointer(_posAttr, 3, GL_FLOAT, GL_FALSE, stride, 0);

    glGenBuffers(1, &_indBuf);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _indBuf);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glBindVertexArray(0);

    rebuildProgram();

    // Locations
    GLint uboIndex = glGetUniformBlockIndex(_program, "Transform");
    if (uboIndex != GL_INVALID_INDEX) {
        glUniformBlockBinding(_program, uboIndex, g_transformBinding);
    }
    _attrPosition = glGetAttribLocation(_program, "Position");

}
void
GLTessQuad::Draw(GLuint xformUB, bool wireframe) const {

    glUseProgram(_program);

    glBindVertexArray(_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _attrBuf);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _indBuf);

    glBindBufferBase(GL_UNIFORM_BUFFER, g_transformBinding, xformUB);

    glEnableVertexAttribArray(_attrPosition);
    glVertexAttribPointer(_attrPosition, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);

    glUniform1fv(glGetUniformLocation(_program, "TessLevelInner"), 2, _tessFactorInner);
    glUniform1fv(glGetUniformLocation(_program, "TessLevelOuter"), 4, _tessFactorOuter);

    if (wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glPointSize(2.0f);
        glLineWidth(1.0f);
    } else {
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    }

    glPatchParameteri(GL_PATCH_VERTICES, 4);
    
    //glDrawElements(GL_PATCHES, 4, GL_UNSIGNED_INT, 0);
    glDrawArrays(GL_PATCHES, 0, 4);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glUseProgram(0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void
GLTessQuad::SetSpacingMode(SpacingMode mode) {
    _spacingMode = mode;
    rebuildProgram();
}

void
GLTessQuad::SetTessFactors(float const * inner, float const * outer) {
    memcpy(_tessFactorInner, inner, 2 * sizeof(float));
    memcpy(_tessFactorOuter, outer, 4 * sizeof(float));
}

float const *
GLTessQuad::GetPositions() const {
    return positions;
}

int const *
GLTessQuad::GetIndices() const {
    return indices;
}

