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

#ifndef TESS_QUAD_H
#define TESS_QUAD_H

#include "../common/glUtils.h"

class GLTessQuad {

public:

    enum SpacingMode {
        FRACTIONAL_ODD = 0,
        FRACTIONAL_EVEN = 1,
        EQUAL = 2
    };

    GLTessQuad();

    ~GLTessQuad();

    void Init(float xofs=0.0f, float yofs=0.0f, float zofs=0.0f);

    void Draw(GLuint xformUB, bool wireframe) const;

    void SetSpacingMode(SpacingMode mode);

    void SetTessFactors(float const * inner, float const * outer);

    float const * GetPositions() const;

    int const * GetIndices() const;


private:

    void rebuildProgram();

    SpacingMode _spacingMode;

    float _tessFactorInner[2],
          _tessFactorOuter[4];

    GLuint _program;

    GLint  _attrPosition;

    GLuint _vao,
           _indBuf,
           _attrBuf,
           _posAttr;
};

#endif // TESS_QUAD_H
