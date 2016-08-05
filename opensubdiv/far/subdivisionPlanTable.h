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

#ifndef OPENSUBDIV3_FAR_SUBDIVISION_PLAN_H
#define OPENSUBDIV3_FAR_SUBDIVISION_PLAN_H

#include "../version.h"

#include "../far/characteristicMap.h"
#include "../far/types.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class TopologyRefiner;

struct SubdivisionPlan {

    SubdivisionPlan(int _numControls, int _firstControl, int _charIndex) :
        numControls(_numControls), firstControl(_firstControl), charIndex(_charIndex) { }

    int numControls,
        firstControl,
        charIndex;
};

typedef std::vector<SubdivisionPlan> SubdivisionPlanVector;

///
///  \brief Stores subdivision plans
///
/// The subdivision plan is a data structure that represents a feature-adaptive
/// subdivisionhierarchy for the face, down to some fixed, maximum depth.
/// Specifically, it comprises:
///   - an optimized quadtree representing the adaptive subdivision hierarchy
///     of the face.
///   - anordered list of stencils for support control points: those required
///     to evaluate limit samples within each subdivided face. Each stencil
///     represents a weighted sum over base vertices
/// The plan for a face depends only on the configuration of elements that can
/// exert an influence on the limit surface within its local domain. This
/// includes the topology of the face and its 1-ring, sharpness tags for
/// incident edges and vertices, and boundary rules.
///
class SubdivisionPlanTable {

public:

    typedef CharacteristicMap::Options Options;

    int GetNumPlans() const {
        return (int)_plans.size();
    }

    SubdivisionPlan const & GetPlan(Index planIndex) const {
        return _plans[planIndex];
    }

    Characteristic const * GetPlanCharacteristic(Index planIndex) const {
        Index charIndex = _plans[planIndex].charIndex;
        return _charmap.GetCharacteristic(charIndex);
    }

    Index GetMeshControlVertexIndex(Index planIndex, LocalIndex supportIndex) const {
        return _controlVertices[_plans[planIndex].firstControl + supportIndex];
    }

    ConstIndexArray GetPlanControlVertices(Index planIndex) const {
        SubdivisionPlan const & plan = _plans[planIndex];
        return ConstIndexArray(
            &_controlVertices[plan.firstControl], plan.numControls);
    }

    CharacteristicMap const & GetCharacteristicMap() const {
        return _charmap;
    }

    int GetNumSubdivisionPlans() const {
        return (int)_plans.size();
    }

    SubdivisionPlanVector const & GetSubdivisionPlans() const {
        return _plans;
    }

    std::vector<Index> const & GetControlVertices() const {
        return _controlVertices;
    }

private:

    friend class CharacteristicMap;

    SubdivisionPlanTable(CharacteristicMap const & charmap);

    static int countNumPlans(TopologyLevel const & coarseLevel, int regFaceSize);

private:

    SubdivisionPlanVector _plans;

    std::vector<Index> _controlVertices;

    CharacteristicMap const & _charmap;
};


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_SUBDIVISION_PLAN_H */
