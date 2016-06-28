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

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class NeighborhoodBuilder;
class StencilTable;
class TopologyLevel;

///
///  \brief Stores topology characteristic plans
///
class CharacteristicMap {

public:

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
    ///  @name change of basis patches
    ///
    /// \anchor change_of_basis_patches
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

    /// \brief Returns the type of end-cap patches
    EndCapType GetEndCapType() const { return EndCapType(_endCapType); }

    /// \brief Writes a GraphViz 'dot' diagraph of all the characteristic trees
    void WriteCharacteristicsDiagraphs(FILE * fout, bool showIndices=true) const;

private:

    friend class CharacteristicMapFactory;

    CharacteristicMap(EndCapType endcaps) :
        _endCapType(endcaps), _localPointStencils(0), _localPointVaryingStencils(0) { }

    // flags
    unsigned int _endCapType:2;

private:

    //
    // Open-addressing hash
    //

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
