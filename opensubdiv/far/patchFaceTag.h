//
//   Copyright 2016 NVIDIA CORPORATION
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

#ifndef OPENSUBDIV3_FAR_PATCH_FACE_TAG_H
#define OPENSUBDIV3_FAR_PATCH_FACE_TAG_H

#include "../version.h"

#include "../far/types.h"
#include "../vtr/level.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class TopologyRefiner;

//  PatchFaceTag
//  A simple struct containing all information gathered about a face that is relevant
//  to constructing a patch for it (some of these enums should probably be defined more
//  as part of PatchTable)
//
//  Like the HbrFace<T>::AdaptiveFlags, this struct aggregates all of the face tags
//  supporting feature adaptive refinement.  For now it is not used elsewhere and can
//  remain local to this implementation, but we may want to move it into a header of
//  its own if it has greater use later.
//
//  Note that several properties being assigned here attempt to do so given a 4-bit
//  mask of properties at the edges or vertices of the quad.  Still not sure exactly
//  what will be done this way, but the goal is to create lookup tables (of size 16
//  for the 4 bits) to quickly determine was is needed, rather than iteration and
//  branching on the edges or vertices.
//
class PatchFaceTag {

public:
    unsigned int hasPatch        : 1, 
                 isRegular       : 1,
                 transitionMask  : 4,
                 boundaryMask    : 4,
                 boundaryIndex   : 2,
                 boundaryCount   : 3,
                 hasBoundaryEdge : 3,
                 isSingleCrease  : 1;

    /// \brief Clears all tags
    void Clear();

    /// \brief Computes the patch tags for the given face
    void ComputeTags(Far::TopologyRefiner const & refiner,
        Index const levelIndex, Index const faceIndex, int maxIsolationLevel,
            bool useSingleCreasePatch, bool useInfSharpPatch,
                bool generateLegacySharpCornerPatches);

    // debug print
    void Print() const;

private:

    bool isPatchEligible(TopologyRefiner const & refiner,
        int levelIndex, Index faceIndex) const;

    bool isPatchSmoothCorner(TopologyRefiner const & refiner,
        int levelIndex, Index faceIndex,
            bool useInfSharpPatch) const;

    bool isPatchRegular(TopologyRefiner const & refiner,
        int levelIndex, Index faceIndex,
            bool useInfSharpPatch, bool generateLegacySharpCornerPatches) const;

    int getRegularPatchBoundaryMask(TopologyRefiner const & refiner,
        int levelIndex, Index faceIndex,
            bool useInfSharpPatch) const;

    int getTransitionMask(TopologyRefiner const & refiner,
        int levelIndex, Index faceIndex) const;

};

typedef std::vector<PatchFaceTag> PatchFaceTagVector;

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv


#endif /* OPENSUBDIV3_FAR_PATCH_FACE_TAG_H */
