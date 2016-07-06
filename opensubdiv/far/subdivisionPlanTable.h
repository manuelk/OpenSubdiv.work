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

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class TopologyRefiner;

struct SubdivisionPlan {
    Index charIndex,
          rootNodeOffset;
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

    /// \brief destructor
    ~SubdivisionPlanTable();

    typedef CharacteristicMap::Options Options;

    /// \brief Returns a plans table (no topology hahsing)
    static SubdivisionPlanTable const *
        Create(TopologyRefiner const & refiner, Options options=Options());

    Characteristic const * GetCharacteristic(Index planIndex) const {
        Index charIndex = _plans[planIndex].charIndex;
        return _charmap->GetCharacteristic(charIndex);
    }

    SubdivisionPlanVector const & GetSubdivisionPlans() const {
        return _plans;
    }

    CharacteristicMap const * GetCharacteristicMap() const {
        return _charmap;
    }

private:

    friend class CharacteristicMap;

    SubdivisionPlanTable(CharacteristicMap const * charmap);

    static int countPlans(TopologyLevel const & coarseLevel, int regFaceSize);

private:

    SubdivisionPlanVector _plans;

    CharacteristicMap const * _charmap;
};


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_SUBDIVISION_PLAN_H */
