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

#ifndef OPENSUBDIV3_OSD_GL_SUBDIVISION_PLAN_TABLE_H
#define OPENSUBDIV3_OSD_GL_SUBDIVISION_PLAN_TABLE_H

#include "../version.h"

#include "../osd/nonCopyable.h"
#include "../osd/opengl.h"
#include "../osd/types.h"
#include "../far/subdivisionPlanTable.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Osd {

class GLSubdivisionPlanTable : private NonCopyable<GLSubdivisionPlanTable> {

public:

    ~GLSubdivisionPlanTable();

    static GLSubdivisionPlanTable * Create(
        Far::SubdivisionPlanTable const & plansTable, void *deviceContext = NULL);


    GLuint GetCharacteristicTreesBuffer() const {
        return _characteristicTreesBuffer;
    }

    GLuint GetSubdivisionPlansBuffer() const {
        return _subdivisionPlansBuffer;
    }

    int GetNumPlans() const {
        return _numPlans;
    }

protected:

    GLSubdivisionPlanTable();

    bool allocate(Far::SubdivisionPlanTable const & plansTable);

    int _numPlans;

    GLuint _characteristicTreesBuffer,
           _subdivisionPlansBuffer;
};

}  // end namespace Osd

}  // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

}  // end namespace OpenSubdiv

#endif  // OPENSUBDIV3_OSD_GL_SUBDIVISION_PLAN_TABLE_H
