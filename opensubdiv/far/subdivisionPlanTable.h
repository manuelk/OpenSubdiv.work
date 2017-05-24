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

#ifndef OPENSUBDIV3_FAR_SUBDIVISION_PLAN_TABLE_H
#define OPENSUBDIV3_FAR_SUBDIVISION_PLAN_TABLE_H

#include "../version.h"

#include "../far/topologyMap.h"
#include "../far/types.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class TopologyRefiner;

struct FacePlan {

    FacePlan(int _numControls, int _firstControl, int _planIndex) :
        numControls(_numControls), firstControl(_firstControl), planIndex(_planIndex) { }

    int numControls,
        firstControl,
        planIndex;
};

typedef std::vector<FacePlan> FacePlanVector;

///
///  \brief Stores subdivision plans
///
/// The subdivision plan table matches each face of a coarse mesh with its
/// unique subbdivision plan in a TopologyMap and the 1-ring of the face.
/// This information is all that is required to perform arbitrary limit
/// surface evaluations.
///
/// Specifically, each face is associated with:
///   - an index to a subdivision plan in the reference TopologyMap
///   - an ordered list of supporting control vertices
///
class SubdivisionPlanTable {

public:

    /// \brief Returns the number of plans in the table
    int GetNumPlans() const {
        return (int)_plans.size();
    }

    /// \brief Returns true if the the plan has no surface (hole)
    bool PlanIsHole(Index planIndex) const {
        return _plans[planIndex].planIndex == INDEX_INVALID;
    }

    /// \brief Returns the plan at index
    FacePlan const & GetPlan(Index planIndex) const {
        return _plans[planIndex];
    }

    /// \brief Returns the subdivision plan associated with the face
    SubdivisionPlan const * GetSubdivisionPlan(Index faceIndex) const {
        Index planIndex = _plans[faceIndex].planIndex;
        return planIndex!=INDEX_INVALID ?
            _topomap.GetSubdivisionPlan(planIndex) : 0;
    }

    /// \brief Returns the index of the vertex in the control mesh corresponding
    /// to the support supportIndex of plan planIndex
    Index GetMeshControlVertexIndex(Index planIndex, LocalIndex supportIndex) const {
        return _controlVertices[_plans[planIndex].firstControl + supportIndex];
    }

    /// \brief Returns the array of control vertex indices for the plan planIndex
    ConstIndexArray GetPlanControlVertices(Index planIndex) const {
        FacePlan const & plan = _plans[planIndex];
        return ConstIndexArray(
            &_controlVertices[plan.firstControl], plan.numControls);
    }

    /// \brief Returns the TopologyMap associated with the plans in the table
    TopologyMap const & GetTopologyMap() const {
        return _topomap;
    }

    /// \brief Returns a vector with all the plans in the table
    FacePlanVector const & GetFacePlans() const {
        return _plans;
    }

    /// \brief Returns a vector with all the base control vertex neighborhoods
    /// of each plan
    std::vector<Index> const & GetControlVertices() const {
        return _controlVertices;
    }

private:

    friend class TopologyMap;

    SubdivisionPlanTable(TopologyMap const & topomap);

    static int countNumPlans(TopologyLevel const & coarseLevel, int regFaceSize);

private:

    FacePlanVector _plans;

    std::vector<Index> _controlVertices;

    TopologyMap const & _topomap;
};


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_SUBDIVISION_PLAN_TABLE_H */
