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

#ifndef OPENSUBDIV3_FAR_CHARACTERISTIC_MAP_FACTORY_H
#define OPENSUBDIV3_FAR_CHARACTERISTIC_MAP_FACTORY_H


#include "../version.h"

#include "../far/characteristicMap.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class CharacteristicTreeBuilder;
class NeighborhoodBuilder;
class TopologyRefiner;

struct PatchFaceTag;
typedef std::vector<PatchFaceTag> PatchFaceTagVector;


///\brief Factory for constructing CharacteristicMaps from topology.
///
/// CharacteristicMapFactory needs 2 elements in order to build characteristic
/// maps:
///     1) a valid TopologyRefiner that has been adaptively refined
///     2) a valid PatchFaceTagVector that has been created from the refiner
///
class CharacteristicMapFactory {

public:

    typedef CharacteristicMap::Options Options;

    /// \brief Factory constructor for CharacteristicMap
    ///
    /// @param refiner              TopologyRefiner from which to generate patches
    ///                             (must be adaptively refined)
    ///
    /// @param options              Options controlling the creation of the table
    ///
    /// @return                     A new instance of PatchTable
    ///
    static CharacteristicMap const * Create(
        TopologyRefiner const & refiner, Options options=Options());

private:

    class Context;

    static void addCharacteristicToHash(
        TopologyLevel const & level, NeighborhoodBuilder & neighborhoodBuilder,
            int faceIndex, int charIndex, int valence, CharacteristicMap * charmap);

    static Index findOrAddCharacteristic(TopologyRefiner const & refiner,
        NeighborhoodBuilder & builder, CharacteristicTreeBuilder & treeBuilder,
            int faceIndex, CharacteristicMap * charmap);
};

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_CHARACTERISTIC_MAP_FACTORY_H */

