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
#include "../far/patchTableFactory.h"
#include "../far/error.h"
#include "../far/patchFaceTag.h"
#include "../far/ptexIndices.h"
#include "../far/topologyRefiner.h"
#include "../vtr/level.h"
#include "../vtr/fvarLevel.h"
#include "../vtr/refinement.h"
#include "../vtr/stackBuffer.h"
#include "../far/endCapBSplineBasisPatchFactory.h"
#include "../far/endCapGregoryBasisPatchFactory.h"
#include "../far/endCapLegacyGregoryPatchFactory.h"

#include <algorithm>
#include <cassert>
#include <cstring>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace {

//  Helpers for compiler warnings and floating point equality tests
#ifdef __INTEL_COMPILER
#pragma warning (push)
#pragma warning disable 1572
#endif

inline bool isSharpnessEqual(float s1, float s2) { return (s1 == s2); }

#ifdef __INTEL_COMPILER
#pragma warning (pop)
#endif

} // namespace anon


namespace Far {

//
//  Trivial anonymous helper functions:
//
static inline void
offsetAndPermuteIndices(Far::Index const indices[], int count,
                        Far::Index offset, int const permutation[],
                        Far::Index result[]) {

    // The patch vertices for boundary and corner patches
    // are assigned index values even though indices will
    // be undefined along boundary and corner edges.
    // When the resulting patch table is going to be used
    // as indices for drawing, it is convenient for invalid
    // indices to be replaced with known good values, such
    // as the first un-permuted index, which is the index
    // of the first vertex of the patch face.
    Far::Index knownGoodIndex = indices[0];

    if (permutation) {
        for (int i = 0; i < count; ++i) {
            if (permutation[i] < 0) {
                result[i] = offset + knownGoodIndex;
            } else {
                result[i] = offset + indices[permutation[i]];
            }
        }
    } else if (offset) {
        for (int i = 0; i < count; ++i) {
            result[i] = offset + indices[i];
        }
    } else {
        std::memcpy(result, indices, count * sizeof(Far::Index));
    }
}

//
// Builder Context
//
// Helper class aggregating transient contextual data structures during
// the creation of a patch table.
// This helps keeping the factory class stateless.
//
// Note : struct members are not re-entrant nor are they intended to be !
//
struct PatchTableFactory::BuilderContext {

public:
    BuilderContext(TopologyRefiner const & refiner, Options options);

    TopologyRefiner const & refiner;

    Options const options;

    PtexIndices const ptexIndices;

public:
    struct PatchTuple {
        PatchTuple()
            : tag(), faceIndex(-1), levelIndex(-1) { }
        PatchTuple(PatchTuple const & p)
            : tag(p.tag), faceIndex(p.faceIndex), levelIndex(p.levelIndex) { }
        PatchTuple(PatchFaceTag const & tag, int faceIndex, int levelIndex)
            : tag(tag), faceIndex(faceIndex), levelIndex(levelIndex) { }

        PatchFaceTag tag;
        int faceIndex;
        int levelIndex;
    };
    typedef std::vector<PatchTuple> PatchTupleVector;

    int gatherBilinearPatchPoints(Index * iptrs,
                                  PatchTuple const & patch,
                                  int fvarChannel = -1);
    int gatherRegularPatchPoints(Index * iptrs,
                                 PatchTuple const & patch,
                                 int fvarChannel = -1);
    template <class END_CAP_FACTORY_TYPE>
    int gatherEndCapPatchPoints(END_CAP_FACTORY_TYPE *endCapFactory,
                                Index * iptrs,
                                PatchTuple const & patch,
                                int fvarChannel = -1);

    // True if face-varying patches need to be generated for this topology
    bool RequiresFVarPatches() const {
        return (! fvarChannelIndices.empty());
    }

    // Counters accumulating each type of patch during topology traversal
    int numRegularPatches;
    int numIrregularPatches;
    int numIrregularBoundaryPatches;

    // Tuple for each patch identified during topology traversal
    PatchTupleVector patches;

    std::vector<int> levelVertOffsets;
    std::vector< std::vector<int> > levelFVarValueOffsets;

    // These are the indices of face-varying channels in the refiner
    // or empty if we are not populating face-varying data.
    std::vector<int> fvarChannelIndices;
};

// Constructor
PatchTableFactory::BuilderContext::BuilderContext(
    TopologyRefiner const & ref, Options opts) :
    refiner(ref), options(opts), ptexIndices(refiner),
    numRegularPatches(0), numIrregularPatches(0),
    numIrregularBoundaryPatches(0) {

    if (options.generateFVarTables) {
        // If client-code does not select specific channels, default to all
        // the channels in the refiner.
        if (options.numFVarChannels==-1) {
            fvarChannelIndices.resize(refiner.GetNumFVarChannels());
            for (int fvc=0;fvc<(int)fvarChannelIndices.size(); ++fvc) {
                fvarChannelIndices[fvc] = fvc; // std::iota
            }
        } else {
            fvarChannelIndices.assign(
                options.fvarChannelIndices,
                options.fvarChannelIndices+options.numFVarChannels);
        }
    }
}

int
PatchTableFactory::BuilderContext::gatherBilinearPatchPoints(
        Index * iptrs, PatchTuple const & patch, int fvarChannel) {

    Vtr::internal::Level const * level = &refiner.getLevel(patch.levelIndex);
    int levelVertOffset = (fvarChannel < 0)
                        ? levelVertOffsets[patch.levelIndex]
                        : levelFVarValueOffsets[fvarChannel][patch.levelIndex];

    ConstIndexArray cvs = (fvarChannel < 0)
                        ?  level->getFaceVertices(patch.faceIndex)
                        :  level->getFaceFVarValues(patch.faceIndex,
                                fvarChannelIndices[fvarChannel]);

    for (int i = 0; i < cvs.size(); ++i) iptrs[i] = levelVertOffset + cvs[i];
    return cvs.size();
}

int
PatchTableFactory::BuilderContext::gatherRegularPatchPoints(
        Index * iptrs, PatchTuple const & patch, int fvarChannel) {

    Vtr::internal::Level const * level = &refiner.getLevel(patch.levelIndex);
    int levelVertOffset = (fvarChannel < 0)
                        ? levelVertOffsets[patch.levelIndex]
                        : levelFVarValueOffsets[fvarChannel][patch.levelIndex];
    int refinerChannel = (fvarChannel < 0)
                       ? fvarChannel
                       : fvarChannelIndices[fvarChannel];

    Index patchVerts[16];

    int bIndex = patch.tag.boundaryIndex;

    int const * permutation = 0;

    if (patch.tag.boundaryCount == 0) {
        static int const permuteRegular[16] =
            { 5, 6, 7, 8, 4, 0, 1, 9, 15, 3, 2, 10, 14, 13, 12, 11 };
        permutation = permuteRegular;
        level->gatherQuadRegularInteriorPatchPoints(
                patch.faceIndex, patchVerts, /*rotation=*/0, refinerChannel);
    } else if (patch.tag.boundaryCount == 1) {
        // Expand boundary patch vertices and rotate to
        // restore correct orientation.
        static int const permuteBoundary[4][16] = {
            { -1, -1, -1, -1, 11, 3, 0, 4, 10, 2, 1, 5, 9, 8, 7, 6 },
            { 9, 10, 11, -1, 8, 2, 3, -1, 7, 1, 0, -1, 6, 5, 4, -1 },
            { 6, 7, 8, 9, 5, 1, 2, 10, 4, 0, 3, 11, -1, -1, -1, -1 },
            { -1, 4, 5, 6, -1, 0, 1, 7, -1, 3, 2, 8, -1, 11, 10, 9 } };
        permutation = permuteBoundary[bIndex];
        level->gatherQuadRegularBoundaryPatchPoints(
                patch.faceIndex, patchVerts, bIndex, refinerChannel);
    } else if (patch.tag.boundaryCount == 2) {
        // Expand corner patch vertices and rotate to
        // restore correct orientation.
        static int const permuteCorner[4][16] = {
            { -1, -1, -1, -1, -1, 0, 1, 4, -1, 3, 2, 5, -1, 8, 7, 6 },
            { -1, -1, -1, -1, 8, 3, 0, -1, 7, 2, 1, -1, 6, 5, 4, -1 },
            { 6, 7, 8, -1, 5, 2, 3, -1, 4, 1, 0, -1, -1, -1, -1, -1 },
            { -1, 4, 5, 6, -1, 1, 2, 7, -1, 0, 3, 8, -1, -1, -1, -1 } };
        permutation = permuteCorner[bIndex];
        level->gatherQuadRegularCornerPatchPoints(
                patch.faceIndex, patchVerts, bIndex, refinerChannel);
    } else {
        assert(patch.tag.boundaryCount <= 2);
    }

    offsetAndPermuteIndices(
        patchVerts, 16, levelVertOffset, permutation, iptrs);
    return 16;
}

template <class END_CAP_FACTORY_TYPE>
int
PatchTableFactory::BuilderContext::
gatherEndCapPatchPoints(
        END_CAP_FACTORY_TYPE *endCapFactory,
        Index * iptrs, PatchTuple const & patch, int fvarChannel) {

    Vtr::internal::Level const * level = &refiner.getLevel(patch.levelIndex);
    int levelVertOffset = (fvarChannel < 0)
                        ? levelVertOffsets[patch.levelIndex]
                        : levelFVarValueOffsets[fvarChannel][patch.levelIndex];
    int refinerChannel = (fvarChannel < 0)
                       ? fvarChannel
                       : fvarChannelIndices[fvarChannel];

    // identify relevant spans around the corner vertices for the irregular patches
    // (this is just a stub for now -- leaving the span "size" to zero, as constructed,
    // indicates to use the full neighborhood)...
    Vtr::internal::Level::VSpan cornerSpans[4];

    ConstIndexArray cvs = endCapFactory->GetPatchPoints(
        level, patch.faceIndex, cornerSpans, levelVertOffset, refinerChannel);

    for (int i = 0; i < cvs.size(); ++i) iptrs[i] = cvs[i];
    return cvs.size();
}

//
//  Reserves tables based on the contents of the PatchArrayVector in the PatchTable:
//
void
PatchTableFactory::allocateVertexTables(
        PatchTable * table, bool hasSharpness) {

    int ncvs = 0, npatches = 0;
    for (int i=0; i<table->GetNumPatchArrays(); ++i) {
        npatches += table->GetNumPatches(i);
        ncvs += table->GetNumControlVertices(i);
    }

    if (ncvs==0 || npatches==0)
        return;

    table->_patchVerts.resize( ncvs );

    table->_paramTable.resize( npatches );

    if (hasSharpness) {
        table->_sharpnessIndices.resize( npatches, Vtr::INDEX_INVALID );
    }
}

//
//  Allocate face-varying tables
//
void
PatchTableFactory::allocateFVarChannels(
        BuilderContext const & context, PatchTable * table) {

    TopologyRefiner const & refiner = context.refiner;

    int npatches = table->GetNumPatchesTotal();

    table->allocateFVarPatchChannels((int)context.fvarChannelIndices.size());

    // Initialize each channel
    for (int fvc=0; fvc<(int)context.fvarChannelIndices.size(); ++fvc) {
        int refinerChannel = context.fvarChannelIndices[fvc];

        Sdc::Options::FVarLinearInterpolation interpolation =
            refiner.GetFVarLinearInterpolation(refinerChannel);

        table->setFVarPatchChannelLinearInterpolation(interpolation, fvc);

        PatchDescriptor::Type type = context.options.triangulateQuads ?
            PatchDescriptor::TRIANGLES : PatchDescriptor::QUADS;

        table->allocateFVarPatchChannelValues(
            PatchDescriptor(type), npatches, fvc);
    }
}

//
//  Populates the PatchParam for the given face, returning
//  a pointer to the next entry
//
PatchParam
PatchTableFactory::computePatchParam(
    BuilderContext const & context,
    int depth, Vtr::Index faceIndex, int boundaryMask,
    int transitionMask) {

    TopologyRefiner const & refiner = context.refiner;

    // Move up the hierarchy accumulating u,v indices to the coarse level:
    int childIndexInParent = 0,
        u = 0,
        v = 0,
        ofs = 1;

    bool nonquad = (refiner.GetLevel(depth).GetFaceVertices(faceIndex).size() != 4);

    for (int i = depth; i > 0; --i) {
        Vtr::internal::Refinement const& refinement  = refiner.getRefinement(i-1);
        Vtr::internal::Level const&      parentLevel = refiner.getLevel(i-1);

        Vtr::Index parentFaceIndex    = refinement.getChildFaceParentFace(faceIndex);
                 childIndexInParent = refinement.getChildFaceInParentFace(faceIndex);

        if (parentLevel.getFaceVertices(parentFaceIndex).size() == 4) {
            switch ( childIndexInParent ) {
                case 0 :                     break;
                case 1 : { u+=ofs;         } break;
                case 2 : { u+=ofs; v+=ofs; } break;
                case 3 : {         v+=ofs; } break;
            }
            ofs = (unsigned short)(ofs << 1);
        } else {
            nonquad = true;
            // If the root face is not a quad, we need to offset the ptex index
            // CCW to match the correct child face
            Vtr::ConstIndexArray children = refinement.getFaceChildFaces(parentFaceIndex);
            for (int j=0; j<children.size(); ++j) {
                if (children[j]==faceIndex) {
                    childIndexInParent = j;
                    break;
                }
            }
        }
        faceIndex = parentFaceIndex;
    }

    Vtr::Index ptexIndex = context.ptexIndices.GetFaceId(faceIndex);
    assert(ptexIndex!=-1);

    if (nonquad) {
        ptexIndex+=childIndexInParent;
        --depth;
    }

    PatchParam param;
    param.Set(ptexIndex, (short)u, (short)v, (unsigned short) depth, nonquad,
               (unsigned short) boundaryMask, (unsigned short) transitionMask);
    return param;
}

//
//  Indexing sharpnesses
//
inline int
assignSharpnessIndex(float sharpness, std::vector<float> & sharpnessValues) {

    // linear search
    for (int i=0; i<(int)sharpnessValues.size(); ++i) {
        if (isSharpnessEqual(sharpnessValues[i], sharpness)) {
            return i;
        }
    }
    sharpnessValues.push_back(sharpness);
    return (int)sharpnessValues.size()-1;
}

//
//  We should be able to use a single Create() method for both the adaptive and uniform
//  cases.  In the past, more additional arguments were passed to the uniform version,
//  but that may no longer be necessary (see notes in the uniform version below)...
//
PatchTable *
PatchTableFactory::Create(TopologyRefiner const & refiner, Options options) {

    if (refiner.IsUniform()) {
        return createUniform(refiner, options);
    } else {
        return createAdaptive(refiner, options);
    }
}

PatchTable *
PatchTableFactory::createUniform(TopologyRefiner const & refiner, Options options) {

    assert(refiner.IsUniform());

    BuilderContext context(refiner, options);

    // ensure that triangulateQuads is only set for quadrilateral schemes
    options.triangulateQuads &= (refiner.GetSchemeType()==Sdc::SCHEME_BILINEAR ||
                                 refiner.GetSchemeType()==Sdc::SCHEME_CATMARK);

    // level=0 may contain n-gons, which are not supported in PatchTable.
    // even if generateAllLevels = true, we start from level 1.

    int maxvalence = refiner.GetMaxValence(),
        maxlevel = refiner.GetMaxLevel(),
        firstlevel = options.generateAllLevels ? 1 : maxlevel,
        nlevels = maxlevel-firstlevel+1;

    PatchDescriptor::Type ptype = PatchDescriptor::NON_PATCH;
    if (options.triangulateQuads) {
        ptype = PatchDescriptor::TRIANGLES;
    } else {
        switch (refiner.GetSchemeType()) {
            case Sdc::SCHEME_BILINEAR :
            case Sdc::SCHEME_CATMARK  : ptype = PatchDescriptor::QUADS; break;
            case Sdc::SCHEME_LOOP     : ptype = PatchDescriptor::TRIANGLES; break;
        }
    }
    assert(ptype!=PatchDescriptor::NON_PATCH);

    //
    //  Create the instance of the table and allocate and initialize its members.
    PatchTable * table = new PatchTable(maxvalence);

    table->_numPtexFaces = context.ptexIndices.GetNumFaces();

    table->reservePatchArrays(nlevels);

    PatchDescriptor desc(ptype);

    // generate patch arrays
    for (int level=firstlevel, poffset=0, voffset=0; level<=maxlevel; ++level) {

        TopologyLevel const & refLevel = refiner.GetLevel(level);

        int npatches = refLevel.GetNumFaces();
        if (refiner.HasHoles()) {
            for (int i = npatches - 1; i >= 0; --i) {
                npatches -= refLevel.IsFaceHole(i);
            }
        }
        assert(npatches>=0);

        if (options.triangulateQuads)
            npatches *= 2;

        table->pushPatchArray(desc, npatches, &voffset, &poffset, 0);
    }

    // Allocate various tables
    allocateVertexTables( table, /*hasSharpness=*/false );

    if (context.RequiresFVarPatches()) {
        allocateFVarChannels(context, table);
    }

    //
    //  Now populate the patches:
    //

    Index          * iptr = &table->_patchVerts[0];
    PatchParam     * pptr = &table->_paramTable[0];
    Index         ** fptr = 0;

    // we always skip level=0 vertices (control cages)
    Index levelVertOffset = refiner.GetLevel(0).GetNumVertices();

    Index * levelFVarVertOffsets = 0;
    if (context.RequiresFVarPatches()) {

        levelFVarVertOffsets = (Index *)alloca(context.fvarChannelIndices.size()*sizeof(Index));
        memset(levelFVarVertOffsets, 0, context.fvarChannelIndices.size()*sizeof(Index));

        fptr = (Index **)alloca(context.fvarChannelIndices.size()*sizeof(Index *));
        for (int fvc=0; fvc<(int)context.fvarChannelIndices.size(); ++fvc) {
            fptr[fvc] = table->getFVarValues(fvc).begin();
        }
    }

    for (int level=1; level<=maxlevel; ++level) {

        TopologyLevel const & refLevel = refiner.GetLevel(level);

        int nfaces = refLevel.GetNumFaces();
        if (level>=firstlevel) {
            for (int face=0; face<nfaces; ++face) {

                if (refiner.HasHoles() && refLevel.IsFaceHole(face)) {
                    continue;
                }

                ConstIndexArray fverts = refLevel.GetFaceVertices(face);
                for (int vert=0; vert<fverts.size(); ++vert) {
                    *iptr++ = levelVertOffset + fverts[vert];
                }

                *pptr++ = computePatchParam(context, level, face, /*boundary*/0, /*transition*/0);

                if (context.RequiresFVarPatches()) {
                    for (int fvc=0; fvc<(int)context.fvarChannelIndices.size(); ++fvc) {
                        int refinerChannel = context.fvarChannelIndices[fvc];

                        ConstIndexArray fvalues = refLevel.GetFaceFVarValues(face, refinerChannel);
                        for (int vert=0; vert<fvalues.size(); ++vert) {
                            assert((levelVertOffset + fvalues[vert]) < (int)table->getFVarValues(fvc).size());
                            fptr[fvc][vert] = levelFVarVertOffsets[fvc] + fvalues[vert];
                        }
                        fptr[fvc]+=fvalues.size();
                    }
                }

                if (options.triangulateQuads) {
                    // Triangulate the quadrilateral: {v0,v1,v2,v3} -> {v0,v1,v2},{v3,v0,v2}.
                    *iptr = *(iptr - 4); // copy v0 index
                    ++iptr;
                    *iptr = *(iptr - 3); // copy v2 index
                    ++iptr;

                    *pptr = *(pptr - 1); // copy first patch param
                    ++pptr;

                    if (context.RequiresFVarPatches()) {
                        for (int fvc=0; fvc<(int)context.fvarChannelIndices.size(); ++fvc) {
                            *fptr[fvc] = *(fptr[fvc]-4); // copy fv0 index
                            ++fptr[fvc];
                            *fptr[fvc] = *(fptr[fvc]-3); // copy fv2 index
                            ++fptr[fvc];
                        }
                    }
                }
            }
        }

        if (options.generateAllLevels) {
            levelVertOffset += refiner.GetLevel(level).GetNumVertices();
            for (int fvc=0; fvc<(int)context.fvarChannelIndices.size(); ++fvc) {
                int refinerChannel = context.fvarChannelIndices[fvc];
                levelFVarVertOffsets[fvc] += refiner.GetLevel(level).GetNumFVarValues(refinerChannel);
            }
        }
    }
    return table;
}

PatchTable *
PatchTableFactory::createAdaptive(TopologyRefiner const & refiner, Options options) {

    assert(! refiner.IsUniform());

    BuilderContext context(refiner, options);

    //
    //  First identify the patches -- accumulating an inventory of
    //  information about each resulting patch:
    //
    identifyAdaptivePatches(context);

    //
    //  Create and initialize the instance of the table:
    //
    int maxValence = refiner.GetMaxValence();

    PatchTable * table = new PatchTable(maxValence);

    table->_numPtexFaces = context.ptexIndices.GetNumFaces();

    //
    //  Now populate the patches:
    //
    populateAdaptivePatches(context, table);

    return table;
}

//
//  Identify all patches required for faces at all levels -- accumulating the number of patches
//  for each type, and retaining enough information for the patch for each face to populate it
//  later with no additional analysis.
//
void
PatchTableFactory::identifyAdaptivePatches(BuilderContext & context) {

    TopologyRefiner const & refiner = context.refiner;

    //
    //  Iterate through the levels of refinement to inspect and tag components with information
    //  relative to patch generation.  We allocate all of the tags locally and use them to
    //  populate the patches once a complete inventory has been taken and all tables appropriately
    //  allocated and initialized:
    //
    //  The first Level may have no Refinement if it is the only level -- similarly the last Level
    //  has no Refinement, so a single level is effectively the last, but with less information
    //  available in some cases, as it was not generated by refinement.
    //
    int reservePatches = refiner.GetNumFacesTotal();
    context.patches.reserve(reservePatches);

    context.levelVertOffsets.push_back(0);
    context.levelFVarValueOffsets.resize(context.fvarChannelIndices.size());
    for (int fvc=0; fvc<(int)context.fvarChannelIndices.size(); ++fvc) {
        context.levelFVarValueOffsets[fvc].push_back(0);
    }

    for (int levelIndex=0; levelIndex<refiner.GetNumLevels(); ++levelIndex) {
        Vtr::internal::Level const * level = &refiner.getLevel(levelIndex);

        context.levelVertOffsets.push_back(
                context.levelVertOffsets.back() + level->getNumVertices());

        for (int fvc=0; fvc<(int)context.fvarChannelIndices.size(); ++fvc) {
            int refinerChannel = context.fvarChannelIndices[fvc];
            context.levelFVarValueOffsets[fvc].push_back(
                context.levelFVarValueOffsets[fvc].back()
                + level->getNumFVarValues(refinerChannel));
        }

        for (int faceIndex = 0; faceIndex < level->getNumFaces(); ++faceIndex) {

            PatchFaceTag patchTag;
            patchTag.Clear();

            if (! patchTag.ComputeTags(refiner, levelIndex, faceIndex,
                context.options.maxIsolationLevel, context.options.useSingleCreasePatch)) {
                continue;
            }

            context.patches.push_back(
                BuilderContext::PatchTuple(patchTag, faceIndex, levelIndex));

            // Count the patches here to simplify subsequent allocation.
            if (patchTag.isRegular) {
                ++context.numRegularPatches;
            } else {
                ++context.numIrregularPatches;
                // For legacy gregory patches we need to know how many
                // irregular patches are also boundary patches.
                if (patchTag.boundaryCount > 0) {
                    ++context.numIrregularBoundaryPatches;
                }
            }
        }
    }
}

//
//  Populate adaptive patches that we've previously identified.
//
void
PatchTableFactory::populateAdaptivePatches(
    BuilderContext & context, PatchTable * table) {

    TopologyRefiner const & refiner = context.refiner;

    // State needed to populate an array in the patch table.
    // Pointers in this structure are initialized after the patch array
    // data buffers have been allocated and are then incremented as we
    // populate data into the patch table. Currently, we'll have at
    // most 3 patch arrays: Regular, Irregular, and IrregularBoundary.
    struct PatchArrayBuilder {
        PatchArrayBuilder()
            : patchType(PatchDescriptor::REGULAR), numPatches(0)
            , iptr(NULL), pptr(NULL), sptr(NULL) { }

        PatchDescriptor::Type patchType;
        int numPatches;

        Far::Index *iptr;
        Far::PatchParam *pptr;
        Far::Index *sptr;
        Vtr::internal::StackBuffer<Far::Index*,1> fptr;

    private:
        // Non-copyable
        PatchArrayBuilder(PatchArrayBuilder const &) {}
        PatchArrayBuilder & operator=(PatchArrayBuilder const &) {return *this;}

    } arrayBuilders[3];
    int R = 0, IR = 1, IRB = 2; // Regular, Irregular, IrregularBoundary

    // Regular patches patches will be packed into the first patch array
    arrayBuilders[R].patchType = PatchDescriptor::REGULAR;
    arrayBuilders[R].numPatches = context.numRegularPatches;
    int numPatchArrays = (arrayBuilders[R].numPatches > 0);

    switch(context.options.GetEndCapType()) {
        case ENDCAP_BSPLINE_BASIS:
            // Irregular patches are converted to bspline basis and
            // will be packed into the same patch array as regular patches
            IR = IRB = R;
            arrayBuilders[R].numPatches += context.numIrregularPatches;
            // Make sure we've counted this array even when
            // there are no regular patches.
            numPatchArrays = (arrayBuilders[R].numPatches > 0);
            break;
        case ENDCAP_GREGORY_BASIS:
            // Irregular patches (both interior and boundary) are converted
            // to Gregory basis and will be packed into an additional patch array
            IR = IRB = numPatchArrays;
            arrayBuilders[IR].patchType = PatchDescriptor::GREGORY_BASIS;
            arrayBuilders[IR].numPatches += context.numIrregularPatches;
            numPatchArrays += (arrayBuilders[IR].numPatches > 0);
            break;
        case ENDCAP_LEGACY_GREGORY:
            // Irregular interior and irregular boundary patches each will be
            // packed into separate additional patch arrays.
            IR = numPatchArrays;
            arrayBuilders[IR].patchType = PatchDescriptor::GREGORY;
            arrayBuilders[IR].numPatches = context.numIrregularPatches
                                         - context.numIrregularBoundaryPatches;
            numPatchArrays += (arrayBuilders[IR].numPatches > 0);

            IRB = numPatchArrays;
            arrayBuilders[IRB].patchType = PatchDescriptor::GREGORY_BOUNDARY;
            arrayBuilders[IRB].numPatches = context.numIrregularBoundaryPatches;
            numPatchArrays += (arrayBuilders[IRB].numPatches > 0);
            break;
        default:
            break;
    }

    // Create patch arrays
    table->reservePatchArrays(numPatchArrays);

    int voffset=0, poffset=0, qoffset=0;
    for (int arrayIndex=0; arrayIndex<numPatchArrays; ++arrayIndex) {
        PatchArrayBuilder & arrayBuilder = arrayBuilders[arrayIndex];
        table->pushPatchArray(PatchDescriptor(arrayBuilder.patchType),
            arrayBuilder.numPatches, &voffset, &poffset, &qoffset );
    }

    // Allocate patch array data buffers
    bool hasSharpness = context.options.useSingleCreasePatch;
    allocateVertexTables(table, hasSharpness);

    if (context.RequiresFVarPatches()) {
        allocateFVarChannels(context, table);
    }

    // Initialize pointers used while populating patch array data buffers
    for (int arrayIndex=0; arrayIndex<numPatchArrays; ++arrayIndex) {
        PatchArrayBuilder & arrayBuilder = arrayBuilders[arrayIndex];

        arrayBuilder.iptr = table->getPatchArrayVertices(arrayIndex).begin();
        arrayBuilder.pptr = table->getPatchParams(arrayIndex).begin();
        if (hasSharpness) {
            arrayBuilder.sptr = table->getSharpnessIndices(arrayIndex);
        }

        if (context.RequiresFVarPatches()) {
            arrayBuilder.fptr.SetSize((int)context.fvarChannelIndices.size());

            for (int fvc=0; fvc<(int)context.fvarChannelIndices.size(); ++fvc) {

                PatchDescriptor desc = table->GetFVarChannelPatchDescriptor(fvc);
                Index pidx = table->getPatchIndex(arrayIndex, 0);
                int ofs = pidx * desc.GetNumControlVertices();
                arrayBuilder.fptr[fvc] = &table->getFVarValues(fvc)[ofs];
            }
        }
    }

    // endcap factories
    // XXX
    EndCapBSplineBasisPatchFactory *endCapBSpline = NULL;
    EndCapGregoryBasisPatchFactory *endCapGregoryBasis = NULL;
    EndCapLegacyGregoryPatchFactory *endCapLegacyGregory = NULL;
    StencilTable *localPointStencils = NULL;
    StencilTable *localPointVaryingStencils = NULL;

    switch(context.options.GetEndCapType()) {
        case ENDCAP_GREGORY_BASIS:
            localPointStencils = new StencilTable(0);
            localPointVaryingStencils = new StencilTable(0);
            endCapGregoryBasis = new EndCapGregoryBasisPatchFactory(
                refiner,
                localPointStencils,
                localPointVaryingStencils,
                context.options.shareEndCapPatchPoints);
            break;
        case ENDCAP_BSPLINE_BASIS:
            localPointStencils = new StencilTable(0);
            localPointVaryingStencils = new StencilTable(0);
            endCapBSpline = new EndCapBSplineBasisPatchFactory(
                refiner,
                localPointStencils,
                localPointVaryingStencils);
            break;
        case ENDCAP_LEGACY_GREGORY:
            endCapLegacyGregory = new EndCapLegacyGregoryPatchFactory(refiner);
            break;
        default:
            break;
    }

    // Populate patch data buffers
    for (int patchIndex=0; patchIndex<(int)context.patches.size(); ++patchIndex) {

        BuilderContext::PatchTuple const & patch = context.patches[patchIndex];
        int boundaryMask = patch.tag.boundaryMask;
        int transitionMask = patch.tag.transitionMask;

        float sharpness = 0;
        if (hasSharpness && patch.tag.isSingleCrease) {
            Vtr::internal::Level const & level = refiner.getLevel(patch.levelIndex);
            int bIndex = patch.tag.boundaryIndex;
                        boundaryMask = (1<<bIndex);
            sharpness = level.getEdgeSharpness(
                (level.getFaceEdges(patch.faceIndex)[bIndex]));
            sharpness = std::min(sharpness,
                (float)(context.options.maxIsolationLevel-patch.levelIndex));
        }

        // Most patches will be packed into the regular patch array
        PatchArrayBuilder * arrayBuilder = &arrayBuilders[R];

        if (patch.tag.isRegular) {
            arrayBuilder->iptr +=
                context.gatherRegularPatchPoints(arrayBuilder->iptr, patch);

        } else {
            // Switch to building the irregular patch array
            arrayBuilder = &arrayBuilders[IR];

            boundaryMask = 0;
            transitionMask = 0;

            // switch endcap patchtype by option
            switch(context.options.GetEndCapType()) {
                case ENDCAP_GREGORY_BASIS:
                    arrayBuilder->iptr +=
                        context.gatherEndCapPatchPoints(
                            endCapGregoryBasis, arrayBuilder->iptr, patch);
                    break;
                case ENDCAP_BSPLINE_BASIS:
                    arrayBuilder->iptr +=
                        context.gatherEndCapPatchPoints(
                            endCapBSpline, arrayBuilder->iptr, patch);
                    break;
                case ENDCAP_LEGACY_GREGORY:
                    // For legacy gregory patches we may need to switch to
                    // the irregular boundary patch array.
                    if (patch.tag.boundaryCount == 0) {
                        arrayBuilder->iptr +=
                            context.gatherEndCapPatchPoints(
                                endCapLegacyGregory, arrayBuilder->iptr, patch);
                    } else {
                        arrayBuilder = &arrayBuilders[IRB];
                        arrayBuilder->iptr +=
                            context.gatherEndCapPatchPoints(
                                endCapLegacyGregory, arrayBuilder->iptr, patch);
                    }
                    break;
                case ENDCAP_BILINEAR_BASIS:
                    // not implemented yet
                    assert(false);
                    break;
                default:
                    // no endcap
                    break;
            }
        }

        *arrayBuilder->pptr++ =
            computePatchParam(context,
                              patch.levelIndex, patch.faceIndex,
                              boundaryMask, transitionMask);

        if (hasSharpness) {
            *arrayBuilder->sptr++ =
                assignSharpnessIndex(sharpness, table->_sharpnessValues);
        }

        if (context.RequiresFVarPatches()) {
            for (int fvc=0; fvc<(int)context.fvarChannelIndices.size(); ++fvc) {
                // For now, fvar data is always bilinear. Eventually, we will
                // inspect the fvar topology and emit patches accordingly.
                BuilderContext::PatchTuple fvarPatch(patch);
                arrayBuilder->fptr[fvc] +=
                    context.gatherBilinearPatchPoints(
                        arrayBuilder->fptr[fvc], fvarPatch, fvc);
            }
        }
    }

    // finalize end patches
    if (localPointStencils && localPointStencils->GetNumStencils() > 0) {
        localPointStencils->finalize();
    } else {
        delete localPointStencils;
        localPointStencils = NULL;
    }

    if (localPointVaryingStencils && localPointVaryingStencils->GetNumStencils() > 0) {
        localPointVaryingStencils->finalize();
    } else {
        delete localPointVaryingStencils;
        localPointVaryingStencils = NULL;
    }

    switch(context.options.GetEndCapType()) {
        case ENDCAP_GREGORY_BASIS:
            table->_localPointStencils = localPointStencils;
            table->_localPointVaryingStencils = localPointVaryingStencils;
            delete endCapGregoryBasis;
            break;
        case ENDCAP_BSPLINE_BASIS:
            table->_localPointStencils = localPointStencils;
            table->_localPointVaryingStencils = localPointVaryingStencils;
            delete endCapBSpline;
            break;
        case ENDCAP_LEGACY_GREGORY:
            endCapLegacyGregory->Finalize(
                table->GetMaxValence(),
                &table->_quadOffsetsTable,
                &table->_vertexValenceTable);
            delete endCapLegacyGregory;
            break;
        default:
            break;
    }
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

