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


#ifndef OPENSUBDIV3_FAR_TOPOLOGY_MAP_H
#define OPENSUBDIV3_FAR_TOPOLOGY_MAP_H

#include "../version.h"

#include "../far/subdivisionPlan.h"
#include "../far/types.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

namespace internal {
    class SubdivisionPlanBuilder;
}

class Neighborhood;
class NeighborhoodBuilder;
class StencilTable;
class SubdivisionPlanTable;
class TopologyLevel;
class TopologyRefiner;

///
///  \brief Hashed dictionary of subdivision plans
///
class TopologyMap {

public:

    typedef SubdivisionPlan::EndCapType EndCapType;

    struct Options {

        Options() :
             hashSize(10000),
             endCapType(SubdivisionPlan::ENDCAP_BSPLINE_BASIS),
             useTerminalNode(true),
             useDynamicIsolation(true),
             generateLegacySharpCornerPatches(true) { }

        /// \brief Get endcap patch type
        EndCapType GetEndCapType() const { return (EndCapType)endCapType; }

        /// \brief Set endcap patch type
        void SetEndCapType(EndCapType e) { endCapType = e; }

        int hashSize;

        unsigned int endCapType           : 3,  ///< Type of end-cap patches
                     useTerminalNode      : 1,  ///< Use "terminal" nodes on patches with single EV
                     useDynamicIsolation  : 1,  ///< Generate data for run-time dynamic isolation

                     // legacy behaviors (default to true)
                     generateLegacySharpCornerPatches : 1; ///< Generate sharp regular patches at smooth corners (legacy)
    };

    /// \brief Constructor
    TopologyMap(Options options=Options());

    //// \brief Returns the map's configuration options
    Options GetOptions() const { return _options; }


    /// \brief Hashes the topology from the refiner mesh into the map
    /// Note : the refiner must be adaptively refined !
    SubdivisionPlanTable const * HashTopology(TopologyRefiner const & refiner);

    /// \brief Reurns the number of subdivision plans in the map
    int GetNumSubdivisionPlans() const {
        return (int)_plans.size();
    }

    /// \brief Returns the subdivision plan at index
    SubdivisionPlan const * GetSubdivisionPlan(Index planIndex) const {
        return _plans[planIndex];
    }

    /// \brief Returns the maximum number of supports in a subdivision plan
    int GetNumMaxSupports() const {
        return _numMaxSupports;
    }

    /// \brief Returns the capacity of the hash map
    int GetHashMapCapacity() const {
        return (int)_plansHash.capacity();
    }

    /// \brief Returns the sum of the subdivision plan trees sizes
    int GetSubdivisionPlanTreeSizeTotal() const;

    /// \brief Returns the type of end-cap patches
    SubdivisionPlan::EndCapType GetEndCapType() const { return SubdivisionPlan::EndCapType(_options.endCapType); }

    /// \brief Writes a GraphViz 'dot' diagraph of all the subdivision plan trees
    void WriteSubdivisionPlansDigraphs(FILE * fout, char const * title, bool showIndices=true) const;

private:

    //
    // Options & flags
    //

    friend class SubdivisionPlanTable;

    Options _options;

    int _numMaxSupports; // maximum number of supports in a subdivision plan

private:

    //
    // Open-addressing hash
    //
    Index findSubdivisionPlan(Neighborhood const & n, int * rotation=0) const;

    void addSubdivisionPlanToHash(
        TopologyLevel const & level, NeighborhoodBuilder & neighborhoodBuilder,
            int faceIndex, int planIndex, int valence);

    std::vector<int> _plansHash;

    std::vector<SubdivisionPlan *> _plans;
};

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_TOPOLOGY_MAP_H */
