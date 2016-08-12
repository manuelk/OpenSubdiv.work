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

#include "../far/patchFaceTag.h"
#include "../far/error.h"
#include "../far/topologyRefiner.h"
#include "../vtr/level.h"

#include <algorithm>
#include <cassert>
#include <cstring>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

void
PatchFaceTag::Clear() {
    std::memset(this, 0, sizeof(*this));
}

void
PatchFaceTag::Print() const {

    printf("isRegular=%d transitionMask=%d, boundaryMask=%d, "
        "boundaryIndex=%d, boundaryCount=%d, hasBoundaryEdge=%d, isSingleCrease=%d",
            isRegular, transitionMask, boundaryMask, boundaryIndex, boundaryCount, hasBoundaryEdge, isSingleCrease);
}

void
PatchFaceTag::AssignBoundaryPropertiesFromEdgeMask(int boundaryEdgeMask) {
    //
    //  The number of rotations to apply for boundary or corner patches varies on both
    //  where the boundary/corner occurs and whether boundary or corner -- so using a
    //  4-bit mask should be sufficient to quickly determine all cases:
    //
    //  Note that we currently expect patches with multiple boundaries to have already
    //  been isolated, so asserts are applied for such unexpected cases.
    //
    //  Is the compiler going to build the 16-entry lookup table here, or should we do
    //  it ourselves?
    //
    hasBoundaryEdge = true;
    boundaryMask = boundaryEdgeMask;

    switch (boundaryEdgeMask) {
        case 0x0:  boundaryCount = 0, boundaryIndex = 0, hasBoundaryEdge = false;  break;  // no boundaries
        case 0x1:  boundaryCount = 1, boundaryIndex = 0;  break;  // boundary edge 0
        case 0x2:  boundaryCount = 1, boundaryIndex = 1;  break;  // boundary edge 1
        case 0x3:  boundaryCount = 2, boundaryIndex = 1;  break;  // corner/crease vertex 1
        case 0x4:  boundaryCount = 1, boundaryIndex = 2;  break;  // boundary edge 2
        case 0x5:  assert(false);                         break;  // N/A - opposite boundary edges
        case 0x6:  boundaryCount = 2, boundaryIndex = 2;  break;  // corner/crease vertex 2
        case 0x7:  assert(false);                         break;  // N/A - three boundary edges
        case 0x8:  boundaryCount = 1, boundaryIndex = 3;  break;  // boundary edge 3
        case 0x9:  boundaryCount = 2, boundaryIndex = 0;  break;  // corner/crease vertex 0
        case 0xa:  assert(false);                         break;  // N/A - opposite boundary edges
        case 0xb:  assert(false);                         break;  // N/A - three boundary edges
        case 0xc:  boundaryCount = 2, boundaryIndex = 3;  break;  // corner/crease vertex 3
        case 0xd:  assert(false);                         break;  // N/A - three boundary edges
        case 0xe:  assert(false);                         break;  // N/A - three boundary edges
        case 0xf:  assert(false);                         break;  // N/A - all boundaries
        default:   assert(false);                         break;
    }
}

void
PatchFaceTag::AssignBoundaryPropertiesFromVertexMask(int boundaryVertexMask) {

    //
    //  This is strictly needed for the irregular case when a vertex is a boundary in
    //  the presence of no boundary edges -- an extra-ordinary face with only one corner
    //  on the boundary.
    //
    //  Its unclear at this point if patches with more than one such vertex are supported
    //  (if so, how do we deal with rotations) so for now we only allow one such vertex
    //  and assert for all other cases.
    //
    assert(hasBoundaryEdge == false);
    boundaryMask = boundaryVertexMask;

    switch (boundaryVertexMask) {
        case 0x0:  boundaryCount = 0;                     break;  // no boundaries
        case 0x1:  boundaryCount = 1, boundaryIndex = 0;  break;  // boundary vertex 0
        case 0x2:  boundaryCount = 1, boundaryIndex = 1;  break;  // boundary vertex 1
        case 0x3:  assert(false);                         break;
        case 0x4:  boundaryCount = 1, boundaryIndex = 2;  break;  // boundary vertex 2
        case 0x5:  assert(false);                         break;
        case 0x6:  assert(false);                         break;
        case 0x7:  assert(false);                         break;
        case 0x8:  boundaryCount = 1, boundaryIndex = 3;  break;  // boundary vertex 3
        case 0x9:  assert(false);                         break;
        case 0xa:  assert(false);                         break;
        case 0xb:  assert(false);                         break;
        case 0xc:  assert(false);                         break;
        case 0xd:  assert(false);                         break;
        case 0xe:  assert(false);                         break;
        case 0xf:  assert(false);                         break;
        default:   assert(false);                         break;
    }
}

bool
PatchFaceTag::ComputeTags(
    Far::TopologyRefiner const & refiner,
        Index const levelIndex, Index const faceIndex,
            int maxIsolationLevel, bool useSingleCreasePatch) {

    Vtr::internal::Level const * level = &refiner.getLevel(levelIndex);

    hasPatch = false;

    if (level->isFaceHole(faceIndex)) {
        return false;
    }

    //
    //  Given components at Level[i], we need to be looking at Refinement[i] -- and not
    //  [i-1] -- because the Refinement has transitional information for its parent edges
    //  and faces.
    //
    //  For components in this level, we want to determine:
    //    - what Edges are "transitional" (already done in Refinement for parent)
    //    - what Faces are "transitional" (already done in Refinement for parent)
    //    - what Faces are "complete" (applied to this Level in previous refinement)
    //
    Vtr::internal::Refinement const * refinement =
        (levelIndex < refiner.GetMaxLevel())
            ? refinement = &refiner.getRefinement(levelIndex) : 0;

    //
    //  This face does not warrant a patch under the following conditions:
    //
    //      - the face was fully refined into child faces
    //      - the face is not a quad (should have been refined, so assert)
    //      - the face is not "complete"
    //
    //  The first is trivially determined, and the second is really redundant.  The
    //  last -- "incompleteness" -- indicates a face that exists to support the limit
    //  of some neighboring component, and which does not have its own neighborhood
    //  fully defined for its limit.  If any child vertex of a vertex of this face is
    //  "incomplete" (and all are tagged) the face must be "incomplete", so get the
    //  "composite" tag which combines bits for all vertices:
    //
    Vtr::internal::Refinement::SparseTag refinedFaceTag =
        refinement
            ? refinement->getParentFaceSparseTag(faceIndex)
            : Vtr::internal::Refinement::SparseTag();

    if (refinedFaceTag._selected) {
        return false;
    }

    Vtr::ConstIndexArray fVerts = level->getFaceVertices(faceIndex);
    assert(fVerts.size() == 4);

    Vtr::internal::Level::VTag compFaceVertTag = level->getFaceCompositeVTag(fVerts);
    if (compFaceVertTag._incomplete) {
        return false;
    }

    //
    //  We have a quad that will be represented as a B-spline or end cap patch.  Use
    //  the "composite" tag again to quickly determine if any vertex is irregular, on
    //  a boundary, non-manifold, etc.
    //
    //  Inspect the edges for boundaries and transitional edges and pack results into
    //  4-bit masks.  We detect boundary edges rather than vertices as we hope to
    //  replace the mask in future with one for infinitely sharp edges -- allowing
    //  us to detect regular patches and avoid isolation.  We still need to account
    //  for the irregular/xordinary case when a corner vertex is a boundary but there
    //  are no boundary edges.
    //
    //  As for transition detection, assign the transition properties (even if 0).
    //
    //  NOTE on patches around non-manifold vertices:
    //      In most cases the use of regular boundary or corner patches is what we want,
    //  but in some, i.e. when a non-manifold vertex is infinitely sharp, using
    //  such patches will create some discontinuities.  At this point non-manifold
    //  support is still evolving and is not strictly defined, so this is left to
    //  a later date to resolve.
    //
    //  NOTE on infinitely sharp (hard) edges:
    //      We should be able to adapt this later to detect hard (inf-sharp) edges
    //  rather than just boundary edges -- there is a similar tag per edge.  That
    //  should allow us to generate regular patches for interior hard features.
    //
    bool hasBoundaryVertex    = compFaceVertTag._boundary;
    bool hasNonManifoldVertex = compFaceVertTag._nonManifold;
    bool hasXOrdinaryVertex   = compFaceVertTag._xordinary;

    hasPatch = true;

    isRegular = ! hasXOrdinaryVertex || hasNonManifoldVertex;

    // single crease patch optimization
    if (useSingleCreasePatch &&
        ! hasXOrdinaryVertex && ! hasBoundaryVertex && ! hasNonManifoldVertex) {

        Vtr::ConstIndexArray fEdges = level->getFaceEdges(faceIndex);
        Vtr::internal::Level::ETag compFaceETag = level->getFaceCompositeETag(fEdges);

        if (compFaceETag._semiSharp || compFaceETag._infSharp) {
            float sharpness = 0;
            int rotation = 0;
            if (level->isSingleCreasePatch(faceIndex, &sharpness, &rotation)) {

                // cap sharpness to the max isolation level
                float cappedSharpness =
                        std::min(sharpness, (float)(maxIsolationLevel - levelIndex));
                if (cappedSharpness > 0) {
            isSingleCrease = true;
            boundaryIndex = rotation;
                }
            }
        }
    }

    //  Identify boundaries for both regular and xordinary patches -- non-manifold
    //  (infinitely sharp) edges and vertices are currently interpreted as boundaries
    //  for regular patches, though an irregular patch or extrapolated boundary patch
    //  is really necessary in future for some non-manifold cases.
    //
    if (hasBoundaryVertex || hasNonManifoldVertex) {
        Vtr::ConstIndexArray fEdges = level->getFaceEdges(faceIndex);

        int boundaryEdgeMask = ((level->getEdgeTag(fEdges[0])._boundary) << 0) |
                               ((level->getEdgeTag(fEdges[1])._boundary) << 1) |
                               ((level->getEdgeTag(fEdges[2])._boundary) << 2) |
                               ((level->getEdgeTag(fEdges[3])._boundary) << 3);
        if (hasNonManifoldVertex) {
            int nonManEdgeMask = ((level->getEdgeTag(fEdges[0])._nonManifold) << 0) |
                                 ((level->getEdgeTag(fEdges[1])._nonManifold) << 1) |
                                 ((level->getEdgeTag(fEdges[2])._nonManifold) << 2) |
                                 ((level->getEdgeTag(fEdges[3])._nonManifold) << 3);

            //  Other than non-manifold edges, non-manifold vertices that were made
            //  sharp should also trigger new "boundary" edges for the sharp corner
            //  patches introduced in these cases.
            //
            if (level->getVertexTag(fVerts[0])._nonManifold &&
                level->getVertexTag(fVerts[0])._infSharp) {
                nonManEdgeMask |= (1 << 0) | (1 << 3);
            }
            if (level->getVertexTag(fVerts[1])._nonManifold &&
                level->getVertexTag(fVerts[1])._infSharp) {
                nonManEdgeMask |= (1 << 1) | (1 << 0);
            }
            if (level->getVertexTag(fVerts[2])._nonManifold &&
                level->getVertexTag(fVerts[2])._infSharp) {
                nonManEdgeMask |= (1 << 2) | (1 << 1);
            }
            if (level->getVertexTag(fVerts[3])._nonManifold &&
                level->getVertexTag(fVerts[3])._infSharp) {
                nonManEdgeMask |= (1 << 3) | (1 << 2);
            }
            boundaryEdgeMask |= nonManEdgeMask;
        }

        if (boundaryEdgeMask) {
            AssignBoundaryPropertiesFromEdgeMask(boundaryEdgeMask);
        } else {
            int boundaryVertMask = ((level->getVertexTag(fVerts[0])._boundary) << 0) |
                                   ((level->getVertexTag(fVerts[1])._boundary) << 1) |
                                   ((level->getVertexTag(fVerts[2])._boundary) << 2) |
                                   ((level->getVertexTag(fVerts[3])._boundary) << 3);

            if (hasNonManifoldVertex) {
                int nonManVertMask = ((level->getVertexTag(fVerts[0])._nonManifold) << 0) |
                                     ((level->getVertexTag(fVerts[1])._nonManifold) << 1) |
                                     ((level->getVertexTag(fVerts[2])._nonManifold) << 2) |
                                     ((level->getVertexTag(fVerts[3])._nonManifold) << 3);
                boundaryVertMask |= nonManVertMask;
            }
            AssignBoundaryPropertiesFromVertexMask(boundaryVertMask);
        }
    }

    //  XXXX (barfowl) -- why are we approximating a smooth x-ordinary corner with
    //  a sharp corner patch?  The boundary/corner points of the regular patch are
    //  not even made colinear to make it smoother.  Something historical here...
    //
    //  So this treatment may become optional in future and is bracketed with a
    //  condition now for that reason.  We approximate x-ordinary smooth corners
    //  with regular B-spline patches instead of using a Gregory patch.  The smooth
    //  corner must be properly isolated from any other irregular vertices (as it
    //  will be at any level > 1) otherwise the Gregory patch is necessary.
    //
    //  This flag to be initialized with a future option... ?
    bool approxSmoothCornerWithRegularPatch = true;

    if (approxSmoothCornerWithRegularPatch) {
        if (!isRegular && (boundaryCount == 2)) {
            //  We may have a sharp corner opposite/adjacent an xordinary vertex --
            //  need to make sure there is only one xordinary vertex and that it
            //  is the corner vertex.
            if (levelIndex > 1) {
                isRegular = true;
            } else {
                int xordVertex = 0;
                int xordCount = 0;
                if (level->getVertexTag(fVerts[0])._xordinary) { xordCount++; xordVertex = 0; }
                if (level->getVertexTag(fVerts[1])._xordinary) { xordCount++; xordVertex = 1; }
                if (level->getVertexTag(fVerts[2])._xordinary) { xordCount++; xordVertex = 2; }
                if (level->getVertexTag(fVerts[3])._xordinary) { xordCount++; xordVertex = 3; }

                if (xordCount == 1) {
                    //  We require the vertex opposite the xordinary vertex be interior:
                    if (! level->getVertexTag(fVerts[(xordVertex + 2) % 4])._boundary) {
                        isRegular = true;
                    }
                }
            }
        }
    }

    //
    //  Now that all boundary features have have been identified and tagged, assign
    //  the transition type for the patch before taking inventory.
    //
    //  Identify and increment counts for regular patches (both non-transitional and
    //  transitional) and extra-ordinary patches (always non-transitional):
    //
    transitionMask = refinedFaceTag._transitional;

    return true;
}

void
PatchFaceTag::IdentifyAdaptivePatches(Far::TopologyRefiner const & refiner,
    int maxIsolationLevel, bool useSingleCreasePatch, PatchFaceTagVector & patchTags) {

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
    patchTags.resize(refiner.GetNumFacesTotal());

    PatchFaceTag * levelPatchTags = &patchTags[0];

    for (int levelIndex = 0; levelIndex < refiner.GetNumLevels(); ++levelIndex) {

        Vtr::internal::Level const * level = &refiner.getLevel(levelIndex);

        for (int faceIndex = 0; faceIndex < level->getNumFaces(); ++faceIndex) {

            PatchFaceTag & patchTag = levelPatchTags[faceIndex];

            patchTag.Clear();
            patchTag.ComputeTags(
                refiner, levelIndex, faceIndex, maxIsolationLevel, useSingleCreasePatch);
        }
        levelPatchTags += level->getNumFaces();
    }
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

