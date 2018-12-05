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

#include "../far/subdivisionPlanTable.h"
#include "../far/topologyMap.h"
#include "../far/topologyRefiner.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

int
SubdivisionPlanTable::countNumPlans(
    TopologyLevel const & coarseLevel, int regFaceSize) {

    int nfaces = coarseLevel.GetNumFaces(),
        nplans = 0;

    // non-quads require 1 plan for each face-vert
    for (int face = 0; face < nfaces; ++face) {
        if (coarseLevel.IsFaceHole(face)) {
            continue;
        }
        ConstIndexArray fverts = coarseLevel.GetFaceVertices(face);
        nplans += fverts.size()==regFaceSize ? 1 : fverts.size();
    }
    return nplans;
}


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

