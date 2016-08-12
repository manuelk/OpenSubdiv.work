//
//   Copyright 2013 Pixar
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

#ifndef OPENSUBDIV3_FAR_PATCH_TABLE_FACTORY_H
#define OPENSUBDIV3_FAR_PATCH_TABLE_FACTORY_H

#include "../version.h"

#include "../far/patchTable.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

//  Forward declarations (for internal implementation purposes):
struct PatchFaceTag;
class PtexIndices;
class TopologyRefiner;

class PatchTableFactory {
public:

    struct Options {

        Options(unsigned int maxIsolation=10) :
             generateAllLevels(false),
             triangulateQuads(false),
             useSingleCreasePatch(false),
             maxIsolationLevel(maxIsolation),
             endCapType(ENDCAP_GREGORY_BASIS),
             shareEndCapPatchPoints(true),
             generateFVarTables(false),
             numFVarChannels(-1),
             fvarChannelIndices(0)
        { }

        /// \brief Get endcap patch type
        EndCapType GetEndCapType() const { return (EndCapType)endCapType; }

        /// \brief Set endcap patch type
        void SetEndCapType(EndCapType e) { endCapType = e; }

        unsigned int generateAllLevels    : 1, ///< Include levels from 'firstLevel' to 'maxLevel' (Uniform mode only)
                     triangulateQuads     : 1, ///< Triangulate 'QUADS' primitives (Uniform mode only)
                     useSingleCreasePatch : 1, ///< Use single crease patch
                     maxIsolationLevel    : 4, ///< Cap adaptive feature isolation to the given level (max. 10)

                     // end-capping
                     endCapType              : 3, ///< EndCapType
                     shareEndCapPatchPoints  : 1, ///< Share endcap patch points among adjacent endcap patches.
                                                  ///< currently only work with GregoryBasis.

                     // face-varying
                     generateFVarTables   : 1;///< Generate face-varying patch tables
        int          numFVarChannels;          ///< Number of channel indices and interpolation modes passed
        int const *  fvarChannelIndices;       ///< List containing the indices of the channels selected for the factory
    };

    /// \brief Factory constructor for PatchTable
    ///
    /// @param refiner              TopologyRefiner from which to generate patches
    ///
    /// @param options              Options controlling the creation of the table
    ///
    /// @return                     A new instance of PatchTable
    ///
    static PatchTable * Create(TopologyRefiner const & refiner,
                               Options options=Options());

private:
    //
    // Private helper structures
    //
    struct BuilderContext;

    //
    //  Methods for allocating and managing the patch table data arrays:
    //
    static PatchTable * createUniform(TopologyRefiner const & refiner,
                                      Options options);

    static PatchTable * createAdaptive(TopologyRefiner const & refiner,
                                       Options options);

    //
    //  High-level methods for identifying and populating patches associated with faces:
    //

    static bool computePatchTag(BuilderContext & context,
                                Index const levelIndex,
                                Index const faceIndex,
                                PatchFaceTag &patchTag);

    static void identifyAdaptivePatches(BuilderContext & context);

    static void populateAdaptivePatches(BuilderContext & context,
                                        PatchTable * table);

    static void allocateVertexTables(PatchTable * table, bool hasSharpness);

    static void allocateFVarChannels(BuilderContext const & context,
                                     PatchTable * table);

    static PatchParam computePatchParam(BuilderContext const & context,
                                        int level, int face,
                                        int boundaryMask, int transitionMask);
};

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv


#endif /* OPENSUBDIV3_FAR_PATCH_TABLE_FACTORY_H */
