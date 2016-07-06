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


#ifndef OPENSUBDIV3_FAR_CHARACTERISTIC_MAP_H
#define OPENSUBDIV3_FAR_CHARACTERISTIC_MAP_H

#include "../version.h"

#include "../far/characteristic.h"
#include "../far/types.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class CharacteristicTreeBuilder;
class Neighborhood;
class NeighborhoodBuilder;
class StencilTable;
class TopologyLevel;
class TopologyRefiner;

struct Plan {

    Plan(int chIndex, int rtNode) : charIndex(chIndex), rootNode(rtNode) { }

    int charIndex,
        rootNode;
};

typedef std::vector<Plan> PlanVector;


///
///  \brief Stores topology characteristic plans
///
class CharacteristicMap {

public:

    struct Options {

        Options() :
             hashSize(0),
             endCapType(ENDCAP_BSPLINE_BASIS),
             useTerminalNode(false) { }

        /// \brief Get endcap patch type
        EndCapType GetEndCapType() const { return (EndCapType)endCapType; }

        /// \brief Set endcap patch type
        void SetEndCapType(EndCapType e) { endCapType = e; }

        int hashSize;

        unsigned int endCapType           : 3, ///< Type of end-cap patches
                     useTerminalNode      : 1; ///< Use "terminal" nodes on patches with single EV
    };

    /// \brief Constructor
    CharacteristicMap(Options options=Options()) :
        _options(options), _localPointStencils(0), _localPointVaryingStencils(0) { }

    //@{
    ///  @name Plans
    ///
    /// \anchor arrays_of_plans
    ///
    /// \brief Hashes the topology from the refiner mesh into the map
    /// Note : the refiner must be adaptively refined !
    void MapTopology(TopologyRefiner const & refiner, PlanVector & plans);

    //@}

    //@{
    ///  @name Characteristics
    ///
    /// \anchor arrays_of_characteristics
    ///
    int GetNumCharacteristics() const {
        return (int)_characteristics.size();
    }

    /// \brief Returns the characteristic for index
    Characteristic const * GetCharacteristic(Index charIndex) const {
        return _characteristics[charIndex];
    }

    //@}

    //@{
    ///  @name Change of basis end-cap patches
    ///
    /// \brief Accessors for change of basis patch points
    ///
    /// \brief Returns the stencil table to get change of basis end-cap
    ///        patch points.
    StencilTable const * GetLocalPointStencilTable() const {
        return _localPointStencils;
    }

    /// \brief Returns the varying stencil table for the change of basis
    ///        end-cap patch points.
    StencilTable const * GetLocalPointVaryingStencilTable() const {
        return _localPointVaryingStencils;
    }
    //@}

    /// \brief Returns the capacity of the hash map
    int GetHashMapCapacity() const {
        return (int)_characteristicsHash.capacity();
    }

    /// \brief Returns the sum of the characteristic trees sizes
    int GetCharacteristicTreeSizeTotal() const;

    /// \brief Returns the type of end-cap patches
    EndCapType GetEndCapType() const { return EndCapType(_options.endCapType); }

    /// \brief Writes a GraphViz 'dot' diagraph of all the characteristic trees
    void WriteCharacteristicsDiagraphs(FILE * fout, bool showIndices=true) const;

    //// \breif Returns the map's configuration options
    Options GetOptions() const { return _options; }

private:

    friend class CharacteristicMapFactory;

    // flags
    Options _options;

private:

    //
    // Open-addressing hash
    //
    Index findCharacteristic(Neighborhood const & n, int * rotation=0) const;

    void addCharacteristicToHash(TopologyLevel const & level,
        NeighborhoodBuilder & neighborhoodBuilder,
             int faceIndex, int charIndex, int valence);

    Index findOrAddCharacteristic(
        TopologyRefiner const & refiner, NeighborhoodBuilder & neighborhoodBuilder,
            CharacteristicTreeBuilder & treeBuilder, int faceIndex);

    std::vector<int> _characteristicsHash;

    std::vector<Characteristic *> _characteristics;

private:

    // endcap stencils
    StencilTable const * _localPointStencils,        // endcap basis conversion stencils
                       * _localPointVaryingStencils; // endcap varying stencils (for convenience)
};

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_CHARACTERISTIC_MAP_H */
