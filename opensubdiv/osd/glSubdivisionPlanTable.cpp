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


#include "../osd/glSubdivisionPlanTable.h"

#include "../far/characteristicMap.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Osd {

GLSubdivisionPlanTable::GLSubdivisionPlanTable() :
    _numPlans(0), _characteristicTreesBuffer(0), _subdivisionPlansBuffer(0) {
}

GLSubdivisionPlanTable::~GLSubdivisionPlanTable() {
    if (_characteristicTreesBuffer) {
        glDeleteBuffers(1, &_characteristicTreesBuffer);
    }
    if (_subdivisionPlansBuffer) {
        glDeleteBuffers(1, &_subdivisionPlansBuffer);
    }
}

GLSubdivisionPlanTable *
GLSubdivisionPlanTable::Create(
    Far::SubdivisionPlanTable const & plansTable, void * /*deviceContext*/) {

    GLSubdivisionPlanTable * instance = new GLSubdivisionPlanTable;
    if (instance->allocate(plansTable)) {
        return instance;
    }
    delete instance;
    return 0;
}

bool
GLSubdivisionPlanTable::allocate(Far::SubdivisionPlanTable const & plansTable) {

    // characteristic trees
    Far::CharacteristicMap const * charmap = plansTable.GetCharacteristicMap();

    glGenBuffers(1, &_characteristicTreesBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _characteristicTreesBuffer );
    int size = charmap->GetCharacteristicTreeSizeTotal();
    glBufferData(GL_SHADER_STORAGE_BUFFER, size, NULL, GL_STATIC_DRAW );
    for (int i=0, offset=0; i<charmap->GetNumCharacteristics(); ++i) {
        Far::Characteristic const * ch = charmap->GetCharacteristic(i);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER,
            offset, ch->GetTreeSize(), ch->GetTreeData());
        offset += ch->GetTreeSize();
    }

    // subdivision plans
    Far::SubdivisionPlanVector const & plans = plansTable.GetSubdivisionPlans();
    glGenBuffers(1, &_subdivisionPlansBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _subdivisionPlansBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        plans.size() * sizeof(Far::SubdivisionPlan), &plans[0], GL_STATIC_DRAW);
    _numPlans = (int)plans.size();
    return true;
}

}  // end namespace Osd

}  // end namespace OPENSUBDIV_VERSION
}  // end namespace OpenSubdiv
