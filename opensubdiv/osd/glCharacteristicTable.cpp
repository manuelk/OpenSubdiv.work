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


#include "../osd/glCharacteristicTable.h"

#include "../far/characteristicMap.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Osd {

GLCharacteristicTable::GLCharacteristicTable() :
    _characteristicTreesBuffer(0), _plansBuffer(0) {
}

GLCharacteristicTable::~GLCharacteristicTable() {
    if (_characteristicTreesBuffer) {
        glDeleteBuffers(1, &_characteristicTreesBuffer);
    }
    if (_plansBuffer) {
        glDeleteBuffers(1, &_plansBuffer);
    }
}

GLCharacteristicTable *
GLCharacteristicTable::Create(
    Far::CharacteristicMap const & charmap,
       Far::PlanVector const & plans,void * /*deviceContext*/) {

    GLCharacteristicTable * instance = new GLCharacteristicTable;
    if (instance->allocate(charmap, plans)) {
        return instance;
    }
    delete instance;
    return 0;
}

bool
GLCharacteristicTable::allocate(
    Far::CharacteristicMap const & charmap, Far::PlanVector const & plans) {

    // characteristic trees
    glGenBuffers(1, &_characteristicTreesBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _characteristicTreesBuffer );
    int size = charmap.GetCharacteristicTreeSizeTotal();
    glBufferData(GL_SHADER_STORAGE_BUFFER, size, NULL, GL_STATIC_DRAW );
    for (int i=0; i<charmap.GetNumCharacteristics(); ++i) {
        Far::Characteristic const * ch = charmap.GetCharacteristic(i);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER,
            ch->GetTreeOffset(), ch->GetTreeSize(), ch->GetTreeData());
    }

    // subdivision plans
    glGenBuffers(1, &_plansBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _plansBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        plans.size() * sizeof(Far::Plan), &plans[0], GL_STATIC_DRAW);
    return true;
}

}  // end namespace Osd

}  // end namespace OPENSUBDIV_VERSION
}  // end namespace OpenSubdiv
