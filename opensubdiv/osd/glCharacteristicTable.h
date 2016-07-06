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

#ifndef OPENSUBDIV3_OSD_GL_CHARACTERISTIC_TABLE_H
#define OPENSUBDIV3_OSD_GL_CHARACTERISTIC_TABLE_H

#include "../version.h"

#include "../osd/nonCopyable.h"
#include "../osd/opengl.h"
#include "../osd/types.h"
#include "../far/characteristicMap.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Osd {

class GLCharacteristicTable : private NonCopyable<GLCharacteristicTable> {

public:

    ~GLCharacteristicTable();

    static GLCharacteristicTable * Create(Far::CharacteristicMap const & charmap,
        Far::PlanVector const & plans, void *deviceContext = NULL);


    GLuint GetCharacteristicTreesBuffer() const {
        return _characteristicTreesBuffer;
    }

protected:

    GLCharacteristicTable();

    bool allocate(
        Far::CharacteristicMap const & charmap, Far::PlanVector const & plans);

    GLuint _characteristicTreesBuffer,
           _plansBuffer;
};

}  // end namespace Osd

}  // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

}  // end namespace OpenSubdiv

#endif  // OPENSUBDIV3_OSD_GL_CHARACTERISTIC_TABLE_H
