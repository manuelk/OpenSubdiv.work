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

class Neighborhood;
class NeighborhoodBuilder;
class StencilTable;
class TopologyLevel;

///
///  \brief Stores topology characteristic plans
///
class CharacteristicMap {

public:

    struct Options {

        Options(unsigned int maxIsolation=10) :
             hashSize(0),
             maxIsolationLevel(maxIsolation),
             endCapType(ENDCAP_BSPLINE_BASIS),
             useSingleCreasePatch(false),
             useTerminalNode(false) { }

        /// \brief Get endcap patch type
        EndCapType GetEndCapType() const { return (EndCapType)endCapType; }

        /// \brief Set endcap patch type
        void SetEndCapType(EndCapType e) { endCapType = e; }

        int hashSize;

        unsigned int maxIsolationLevel    : 4, ///< Cap adaptive feature isolation to the given level (max. 10)
                     endCapType           : 3, ///< Type of end-cap patches
                     useSingleCreasePatch : 1, ///< Use single crease patch
                     useTerminalNode      : 1; ///< Use "terminal" nodes on patches with single EV
    };


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
    ///  @name Change of basis patches
    ///
    /// \brief Accessors for change of basis patch points
    ///
    /// \brief Returns the stencil table to get change of basis patch points.
    StencilTable const * GetLocalPointStencilTable() const {
        return _localPointStencils;
    }

    /// \brief Returns the varying stencil table for the change of basis patch
    ///        points.
    StencilTable const * GetLocalPointVaryingStencilTable() const {
        return _localPointVaryingStencils;
    }
    //@}

    /// \brief Returns the capacity of the hash map
    int GetHashMapCapacity() const {
        return (int)_characteristicsHash.capacity();
    }

    /// \brief Returns the sum of the characteristic trees sizes
    int GetCharacteristicTreesSize() const;

    /// \brief Returns the type of end-cap patches
    EndCapType GetEndCapType() const { return EndCapType(_options.endCapType); }

    /// \brief Writes a GraphViz 'dot' diagraph of all the characteristic trees
    void WriteCharacteristicsDiagraphs(FILE * fout, bool showIndices=true) const;

private:

    friend class CharacteristicMapFactory;

    CharacteristicMap(Options options) : _options(options) { }

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
