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

#include "../far/CharacteristicBuilder.h"
#include "../far/characteristicMap.h"
#include "../far/endCapBilinearBasisPatchFactory.h"
#include "../far/endCapBSplineBasisPatchFactory.h"
#include "../far/endCapGregoryBasisPatchFactory.h"
#include "../far/neighborhood.h"
#include "../far/patchFaceTag.h"
#include "../far/stencilTableFactory.h"
#include "../far/topologyRefinerFactory.h"
#include "../vtr/level.h"
#include "../vtr/fvarLevel.h"
#include "../vtr/refinement.h"

#include <cassert>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

using Vtr::internal::Refinement;
using Vtr::internal::Level;
using Vtr::internal::FVarLevel;

namespace internal {

inline Level::ETag
getSingularEdgeMask(bool includeAllInfSharpEdges = false) {

    Level::ETag eTagMask;
    eTagMask.clear();
    eTagMask._boundary = true;
    eTagMask._nonManifold = true;
    eTagMask._infSharp = includeAllInfSharpEdges;
    return eTagMask;
}

inline bool
isEdgeSingular(Level const & level, FVarLevel const * fvarLevel, Index eIndex,
               Level::ETag eTagMask)
{
    Level::ETag eTag = level.getEdgeTag(eIndex);
    if (fvarLevel) {
        eTag = fvarLevel->getEdgeTag(eIndex).combineWithLevelETag(eTag);
    }

    Level::ETag::ETagSize * iTag  = reinterpret_cast<Level::ETag::ETagSize*>(&eTag);
    Level::ETag::ETagSize * iMask = reinterpret_cast<Level::ETag::ETagSize*>(&eTagMask);
    return (*iTag & *iMask) > 0;
}

void
identifyManifoldCornerSpan(Level const & level, Index fIndex,
                           int fCorner, Level::ETag eTagMask,
                           Level::VSpan & vSpan, int fvc = -1)
{
    FVarLevel const * fvarLevel = (fvc < 0) ? 0 : &level.getFVarLevel(fvc);

    ConstIndexArray fVerts = level.getFaceVertices(fIndex);
    ConstIndexArray fEdges = level.getFaceEdges(fIndex);

    ConstIndexArray vEdges = level.getVertexEdges(fVerts[fCorner]);
    int             nEdges = vEdges.size();

    int iLeadingStart  = vEdges.FindIndex(fEdges[fCorner]);
    int iTrailingStart = (iLeadingStart + 1) % nEdges;

    vSpan.clear();
    vSpan._numFaces = 1;

    int iLeading  = iLeadingStart;
    while (! isEdgeSingular(level, fvarLevel, vEdges[iLeading], eTagMask)) {
        ++vSpan._numFaces;
        iLeading = (iLeading + nEdges - 1) % nEdges;
        if (iLeading == iTrailingStart) break;
    }

    int iTrailing = iTrailingStart;
    while (! isEdgeSingular(level, fvarLevel, vEdges[iTrailing], eTagMask)) {
        ++vSpan._numFaces;
        iTrailing = (iTrailing + 1) % nEdges;
        if (iTrailing == iLeadingStart) break;
    }
    vSpan._startFace = (LocalIndex) iLeading;
}

void
identifyNonManifoldCornerSpan(Level const & level, Index fIndex,
                              int fCorner, Level::ETag /* eTagMask */,
                              Level::VSpan & vSpan, int /* fvc */ = -1)
{
    //  For now, non-manifold patches revert to regular patches -- just identify
    //  the single face now for a sharp corner patch.
    //
    //  Remember that the face may be incident the vertex multiple times when
    //  non-manifold, so make sure the local index of the corner vertex in the
    //  face identified additionally matches the corner.
    //
    //FVarLevel const * fvarLevel = (fvc < 0) ? 0 : &level.getFVarChannel(fvc);

    Index vIndex = level.getFaceVertices(fIndex)[fCorner];

    ConstIndexArray      vFaces  = level.getVertexFaces(vIndex);
    ConstLocalIndexArray vInFace = level.getVertexFaceLocalIndices(vIndex);

    vSpan.clear();
    for (int i = 0; i < vFaces.size(); ++i) {
        if ((vFaces[i] == fIndex) && ((int)vInFace[i] == fCorner)) {
            vSpan._startFace = (LocalIndex) i;
            vSpan._numFaces = 1;
            vSpan._sharp = true;
            break;
        }
    }
    assert(vSpan._numFaces == 1);
}

//
// Helper class to keep track of end-cap stencils
//
struct EndCapBuilder {

    EndCapBuilder(TopologyRefiner const & refiner, EndCapType t) :
        type(t) {

        // create stencil tables for end caps w/ matching factory
        endcapStencils = new StencilTable(0);
        endcapVaryingStencils = new StencilTable(0);

        switch (type) {
            case ENDCAP_BILINEAR_BASIS:
                bilinearBasis = new EndCapBilinearBasisPatchFactory(
                    refiner, endcapStencils, endcapVaryingStencils);
                break;
            case ENDCAP_BSPLINE_BASIS:
                bsplineBasis = new EndCapBSplineBasisPatchFactory(
                    refiner, endcapStencils, endcapVaryingStencils);
                break;
            case ENDCAP_GREGORY_BASIS:
                gregoryBasis = new EndCapGregoryBasisPatchFactory(
                    refiner, endcapStencils, endcapVaryingStencils);
                break;
            default:
                break;
        }
    }

    EndCapBuilder::~EndCapBuilder() {
        switch (type) {
            case ENDCAP_BILINEAR_BASIS:
                delete bilinearBasis;
                break;
            case ENDCAP_BSPLINE_BASIS:
                delete bsplineBasis;
                break;
            case ENDCAP_GREGORY_BASIS:
                delete gregoryBasis;
                break;
            default:
                assert(0);
        }
    }

    void GetIrregularPatchCornerSpans(Vtr::internal::Level const & level,
        Index faceIndex, bool useInfSharpPatch, bool generateLegacySharpCornerPatches,
            Level::VSpan cornerSpans[4]) {

        //  Retrieve tags and identify other information for the corner vertices:
        Level::VTag vTags[4];
        level.getFaceVTags(faceIndex, vTags);

        //
        //  For each corner vertex, use the complete neighborhood when possible (which
        //  does not require a search, otherwise identify the span of interest around
        //  the vertex:
        //
        ConstIndexArray fVerts = level.getFaceVertices(faceIndex);

        Level::ETag singularEdgeMask = getSingularEdgeMask(useInfSharpPatch);

        for (int i = 0; i < fVerts.size(); ++i) {
            bool testInfSharp = useInfSharpPatch &&
               (vTags[i]._infSharpEdges && (vTags[i]._rule != Sdc::Crease::RULE_DART));

            if (!testInfSharp) {
                cornerSpans[i].clear();
            } else {
                if (!vTags[i]._nonManifold) {
                    identifyManifoldCornerSpan(
                            level, faceIndex, i, singularEdgeMask, cornerSpans[i]);
                } else {
                    identifyNonManifoldCornerSpan(
                            level, faceIndex, i, singularEdgeMask, cornerSpans[i]);
                }
            }
            if (vTags[i]._corner) {
                cornerSpans[i]._sharp = true;
            } else if (useInfSharpPatch) {
                cornerSpans[i]._sharp = vTags[i]._infIrregular && (vTags[i]._rule == Sdc::Crease::RULE_CORNER);
            }

            //  Legacy option -- reinterpret an irregular smooth corner as sharp if specified:
            if (!cornerSpans[i]._sharp && generateLegacySharpCornerPatches) {
                if (vTags[i]._xordinary && vTags[i]._boundary && !vTags[i]._nonManifold) {
                        int nFaces = cornerSpans[i].isAssigned() ? cornerSpans[i]._numFaces
                                   : level.getVertexFaces(fVerts[i]).size();
                        cornerSpans[i]._sharp = (nFaces == 1);
                }
            }
        }
    }

    ConstIndexArray GatherPatchPoints(Vtr::internal::Level const & level,
        Index faceIndex, int levelVertOffset,
            bool useInfSharpPatch, bool generateLegacySharpCornerPatches) {

        ConstIndexArray cvs;

        Level::VSpan irregCornerSpans[4];

        GetIrregularPatchCornerSpans(level, faceIndex,
            useInfSharpPatch, generateLegacySharpCornerPatches, irregCornerSpans);

        switch (type) {
            case ENDCAP_BILINEAR_BASIS:
                cvs = bilinearBasis->GetPatchPoints(
                    &level, faceIndex, irregCornerSpans, levelVertOffset);
                break;
            case ENDCAP_BSPLINE_BASIS:
                cvs = bsplineBasis->GetPatchPoints(
                    &level, faceIndex, irregCornerSpans, levelVertOffset);
                break;
            case ENDCAP_GREGORY_BASIS:
                cvs = gregoryBasis->GetPatchPoints(
                    &level, faceIndex, irregCornerSpans, levelVertOffset);
                break;
            default:
                assert(0);
        }
        return cvs;
    }

    StencilTable const * FinalizeStencils() {
        if (endcapStencils && (endcapStencils->GetNumStencils() > 0)) {
            endcapStencils->finalize();
        } else {
            delete endcapStencils;
            endcapStencils = nullptr;
        }
        return endcapStencils;
    }

    EndCapType type;

    union {
        EndCapBilinearBasisPatchFactory * bilinearBasis;
        EndCapBSplineBasisPatchFactory * bsplineBasis;
        EndCapGregoryBasisPatchFactory * gregoryBasis;
    };

    StencilTable * endcapStencils;
    StencilTable * endcapVaryingStencils;
};

//
//  Helper functions:
//

inline bool isSharpnessEqual(float s1, float s2) { return (s1 == s2); }

static inline void
offsetAndPermuteIndices(Index const indices[], int count,
                        Index offset, int const permutation[],
                        Index result[]) {

    // The patch vertices for boundary and corner patches
    // are assigned index values even though indices will
    // be undefined along boundary and corner edges.
    // When the resulting patch table is going to be used
    // as indices for drawing, it is convenient for invalid
    // indices to be replaced with known good values, such
    // as the first un-permuted index, which is the index
    // of the first vertex of the patch face.
    Index knownGoodIndex = indices[0];

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
        std::memcpy(result, indices, count * sizeof(Index));
    }
}

// convert CCW winding to match bitwise ^= traversal
//
//  Sequential      ^ Bitwise     traversal pseudo-code
//  +---+---+       +---+---+     corner = 0
//  | 3 | 2 |       | 2 | 3 |     while (recursive)
//  +---+---+  ==>  +---+---+         if s>0.5 then corner~=1; s=1-s;
//  | 0 | 1 |       | 0 | 1 |         if t>0.5 then corner~=2; t=1-t;
//  +---+---+       +---+---+         s *= 2
//                                    t *= 2
static LocalIndex
permuteWinding(LocalIndex i) {
   static int const permuteWinding[4] = { 0, 1, 3, 2 };
   return permuteWinding[i];
}

// Terminal node helpers : copy indices from 16-wide bicubic basis into

// 25-wide terminal node. ('X' = extraordinary vertex)
// evIndex
//         0           1            2            3
//    X . . . .    . . . . X    + + + + .    . + + + +
//    . + + + +    + + + + .    + + + + .    . + + + +
//    . + + + +    + + + + .    + + + + .    . + + + +
//    . + + + +    + + + + .    + + + + .    . + + + +
//    . + + + +    + + + + .    . . . . X    X . . . .
inline void
copyDiagonalIndices(int evIndex, Index const * src, Index * dst) {
    static int offsets[4] = { 6, 5, 0, 1 };
    Index * cornerPtr = dst + offsets[evIndex];
    memcpy(cornerPtr + 0,  src + 0,  4 * sizeof(Index));
    memcpy(cornerPtr + 5,  src + 4,  4 * sizeof(Index));
    memcpy(cornerPtr + 10, src + 8,  4 * sizeof(Index));
    memcpy(cornerPtr + 15, src + 12, 4 * sizeof(Index));
}

// evIndex
//        0            1            2            3
//    X + + + +    + + + + X    . . . . .    . . . . .
//    . . . . .    . . . . .    . . . . .    . . . . .
//    . . . . .    . . . . .    . . . . .    . . . . .
//    . . . . .    . . . . .    . . . . .    . . . . .
//    . . . . .    . . . . .    + + + + X    X + + + +
inline void
copyRowIndices(int evIndex, Index const * src, Index * dst) {
    static int srcOffsets[4] = { 0, 0, 12, 12 },
               dstOffsets[4] = { 1, 0, 20, 21 };
    src += srcOffsets[evIndex];
    dst += dstOffsets[evIndex];
    memcpy(dst, src, 4 * sizeof(Index));
}

// evIndex
//        0            1            2            3
//    X . . . .    . . . . X    . . . . +    + . . . .
//    + . . . .    . . . . +    . . . . +    + . . . .
//    + . . . .    . . . . +    . . . . +    + . . . .
//    + . . . .    . . . . +    . . . . +    + . . . .
//    + . . . .    . . . . +    . . . . X    X . . . .
inline void
copyColIndices(int evIndex, Index const * src, Index * dst) {
    static int srcOffsets[4] = { 0, 3, 3, 0 },
               dstOffsets[4] = { 5, 9, 4, 0 };
    for (int i=0; i<4; ++i, src+=4, dst+=5) {
        *(dst+dstOffsets[evIndex]) = *(src+srcOffsets[evIndex]);
    }
}

static StencilTable const *
generateStencilTable(
    TopologyRefiner const & refiner, EndCapBuilder & endcapBuilder) {

    StencilTableFactory::Options options;
    options.generateOffsets = true;
    options.generateControlVerts = true;
    options.generateIntermediateLevels = true;

    StencilTable const * regularStencils = 0,
                       * localPointStencils = 0,
                       * result = 0;

    regularStencils =  StencilTableFactory::Create(refiner, options),

    localPointStencils = endcapBuilder.FinalizeStencils();

    if (regularStencils && localPointStencils) {
        // concatenate & factorize end-cap stencils
        result = StencilTableFactory::AppendLocalPointStencilTable(
            refiner, regularStencils, localPointStencils);
        delete localPointStencils;
        delete regularStencils;
    } else {
        result = regularStencils;
    }
    return result;
}

//
// Characteristic builder implementation
//

typedef Characteristic::NodeDescriptor NodeDescriptor;

inline CharacteristicBuilder::ProtoNode const &
CharacteristicBuilder::getNodeChild(ProtoNode const & pn, short childIndex) const {
    return const_cast<CharacteristicBuilder *>(this)->getNodeChild(pn, childIndex);
}

inline CharacteristicBuilder::ProtoNode &
CharacteristicBuilder::getNodeChild(ProtoNode const & pn, short childIndex) {
    return _nodeStore[pn.levelIndex+1][pn.children[childIndex]];
}

CharacteristicBuilder::CharacteristicBuilder(TopologyRefiner const & refiner,
    CharacteristicMap const & charmap) :
        _refiner(refiner),
        _charmap(charmap) {

    static int const levelSizes[numLevelMax] =
        { 1, 4, 16, 64, 128, 128, 128, 128, 128, 128, 128 };

    _levelVertOffsets.resize(_refiner.GetNumLevels(),0);

    int numLevels = refiner.GetNumLevels();

    for (int level=0, levelVertOffset = 0; level<numLevels; ++level) {

        _nodeStore[level].reserve(levelSizes[level]);

        _levelVertOffsets[level] = levelVertOffset;

        levelVertOffset += _refiner.GetLevel(level).GetNumVertices();
    }

    // worse case : every face has a new unique topology - likely, most of
    // these contexts will never be used
    _buildContexts.reserve(_refiner.GetLevel(0).GetNumFaces());

    _endcapBuilder = new EndCapBuilder(refiner, getEndCapType());
}

CharacteristicBuilder::~CharacteristicBuilder() {
    delete _endcapBuilder;
}

bool
CharacteristicBuilder::computeSubPatchDomain(
    int levelIndex, Index faceIndex, short * s, short * t) const {

    // Move up the hierarchy accumulating u,v indices to the coarse level:
    int childIndexInParent = 0,
        u = 0,
        v = 0,
        ofs = 1;

    bool nonquad = (_refiner.GetLevel(levelIndex).GetFaceVertices(faceIndex).size() != 4);

    for (int i = levelIndex; i > 0; --i) {

        Vtr::internal::Refinement const& refinement  = _refiner.getRefinement(i-1);
        Vtr::internal::Level const&      parentLevel = _refiner.getLevel(i-1);

        Vtr::Index parentFaceIndex  = refinement.getChildFaceParentFace(faceIndex);
                 childIndexInParent = refinement.getChildFaceInParentFace(faceIndex);

        if (parentLevel.getFaceVertices(parentFaceIndex).size() == 4) {
            switch (childIndexInParent) {
                case 0 :                     break; // CCW winding
                case 1 : { u+=ofs;         } break;
                case 2 : { u+=ofs; v+=ofs; } break;
                case 3 : {         v+=ofs; } break;
            }
            ofs = (unsigned short)(ofs << 1);
        } else {
            nonquad = true;
        }
        faceIndex = parentFaceIndex;
    }

    assert(s && t);
    *s = u;
    *t = v;
    return nonquad;
}


void
CharacteristicBuilder::resetNodeStore() {
    for (short level=0; level<numLevelMax; ++level) {
        _nodeStore[level].clear();
    }
}

int
CharacteristicBuilder::identifyNode(int levelIndex, Index faceIndex) {

    ProtoNode node;

    node.active = true;
    node.levelIndex = levelIndex;
    node.faceIndex = faceIndex;

    node.patchTag.Clear();

    node.patchTag.ComputeTags(_refiner,
        levelIndex, faceIndex, getMaxIsolationLevel(),
            useSingleCreasePatches(), useInfSharpPatches(),
                useLegacySharpCornerPatches());

    if (node.patchTag.hasPatch) {
        node.nodeType = node.patchTag.isRegular ?
            Characteristic::NODE_REGULAR : Characteristic::NODE_END;
        node.numChildren = 0;
    } else {
        ConstIndexArray children =
            _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);
        for (int i=0; i<children.size(); ++i) {
            node.children[i] = identifyNode(levelIndex+1, children[i]);
        }
        node.nodeType = Characteristic::NODE_RECURSIVE;
        node.numChildren = (int)children.size();
    }

    _nodeStore[levelIndex].push_back(node);
    return (int)_nodeStore[levelIndex].size()-1;
}

bool
CharacteristicBuilder::nodeIsTerminal(ProtoNode const & pn, int * evIndex) const {

    if (pn.numChildren!=4) {
        return false;
    }

    int regular = 0, irregular = 0;
    for (int i=0; i<(int)pn.numChildren; ++i) {

        ProtoNode const & child = getNodeChild(pn, i);
        if (child.patchTag.isRegular) {
            assert(child.patchTag.hasPatch);
            ++regular;
        } else {
            // trivial rejection for boundaries or creases
            if ((child.patchTag.boundaryCount>0) ||
                 child.patchTag.isSingleCrease) {
                 return false;
            }
            // complete check
            Vtr::internal::Level const * level = &_refiner.getLevel(pn.levelIndex);
            Vtr::ConstIndexArray fverts = level->getFaceVertices(pn.faceIndex);
            assert(fverts.size() == 4);
            Vtr::internal::Level::VTag vt = level->getFaceCompositeVTag(fverts);
            if (vt._semiSharp || vt._semiSharpEdges || vt._rule!=Sdc::Crease::RULE_SMOOTH) {
                return false;
            }
            irregular = i;
        }
    }
    if (regular==3) {
        assert(evIndex);
        *evIndex = irregular;
        return true;
    }
    return false;
}

void
CharacteristicBuilder::identifyTerminalNodes() {

    for (short level=0; level<getMaxIsolationLevel(); ++level) {
        for (int pnIndex=0; pnIndex<(int)_nodeStore[level].size(); ++pnIndex) {
            ProtoNode & pn = _nodeStore[level][pnIndex];
            int evIndex=INDEX_INVALID;
            if (nodeIsTerminal(pn, &evIndex)) {
                assert(evIndex!=INDEX_INVALID);
                for (int i=0; i<(int)pn.numChildren; ++i) {
                    // de-activate regular child nodes that the terminal node replaces
                    ProtoNode & child = getNodeChild(pn, i);
                    child.active = (i==evIndex) ? true : false;
                }
                pn.evIndex = evIndex;
                pn.nodeType = Characteristic::NODE_TERMINAL;
            }
        }
    }
}

int
CharacteristicBuilder::computeNumSupports(
    Characteristic::NodeType nodeType, bool useDynamicIsolation) const {

    typedef Characteristic::Node Node;

    if (nodeType == Characteristic::NODE_REGULAR) {
        return 16;
    } else if (nodeType == Characteristic::NODE_END) {
        return Node::getNumEndCapSupports(getEndCapType());
    } else if (nodeType == Characteristic::NODE_RECURSIVE) {
        return useDynamicIsolation ?
            Node::getNumEndCapSupports(getEndCapType()) : 0;
    } else {
        int nsupports = 0;
        if (nodeType == Characteristic::NODE_TERMINAL) {
            nsupports += 25;
        }
        if (useDynamicIsolation) {
            nsupports += Node::getNumEndCapSupports(getEndCapType());
        }
        return nsupports;
    }
    assert(0);
    return 0;
}

void
CharacteristicBuilder::computeNodeOffsets(
    int * treeSizeOut, short * numSupportsOut, int * numSupportsTotalOut) {

    typedef Characteristic::Node Node;

    int numEndCapSupports = Node::getNumEndCapSupports(getEndCapType());

    int treeSize = 0, numSupports = 0;
    for (short level=0; level<_refiner.GetNumLevels(); ++level) {

        for (int pnIndex=0; pnIndex<(int)_nodeStore[level].size(); ++pnIndex) {

            ProtoNode & pn = _nodeStore[level][pnIndex];

            if (!pn.active) {
                continue;
            }

            pn.treeOffset = treeSize;
            pn.firstSupport = numSupports;

            Characteristic::NodeType nodeType =
                (Characteristic::NodeType)pn.nodeType;

            treeSize += Node::getNodeSize(
                nodeType, (bool)pn.patchTag.isSingleCrease);

            numSupports += computeNumSupports(
                nodeType, useDynamicIsolation());
        }
        numSupportsOut[level] = numSupports;
    }
    *treeSizeOut = treeSize;
    *numSupportsTotalOut = numSupports;
}

void
CharacteristicBuilder::populateRegularNode(
    ProtoNode const & pn, int * treePtr, Index * supportsPtr) const {

    Level const & level = _refiner.getLevel(pn.levelIndex);

    Index patchVerts[16];

    int const * permutation = 0;
    bool singleCrease = false;
    float sharpness = 0.f;

    int bType = 0,
        bIndex = pn.patchTag.boundaryIndex,
        boundaryMask = pn.patchTag.isSingleCrease ? 0 : pn.patchTag.boundaryMask,
        levelVertOffset = _levelVertOffsets[pn.levelIndex];

    if (boundaryMask) {
        static int const boundaryEdgeMaskToType[16] =
            { 0, 1, 1, 2, 1, -1, 2, -1, 1, 2, -1, -1, 2, -1, -1, -1 };
        static int const boundaryEdgeMaskToFeature[16] =
            { -1, 0, 1, 1, 2, -1, 2, -1, 3, 0, -1, -1, 3, -1, -1, -1 };

        bType  = boundaryEdgeMaskToType[boundaryMask];
        bIndex = boundaryEdgeMaskToFeature[boundaryMask];
    }

    if (bType == 0) {
        static int const permuteRegular[16] = { 5, 6, 7, 8, 4, 0, 1, 9, 15, 3, 2, 10, 14, 13, 12, 11 };
        permutation = permuteRegular;
        level.gatherQuadRegularInteriorPatchPoints(pn.faceIndex, patchVerts, 0);
        if (pn.patchTag.isSingleCrease) {
            int maxIsolation = _refiner.GetAdaptiveOptions().isolationLevel;
            singleCrease = true;
            boundaryMask = 1 << bIndex;
            sharpness = level.getEdgeSharpness((level.getFaceEdges(pn.faceIndex)[bIndex]));
            sharpness = std::min(sharpness, (float)(maxIsolation-pn.levelIndex));
        }
    } else if (bType == 1) {
        // Expand boundary patch vertices and rotate to restore correct orientation.
        static int const permuteBoundary[4][16] = {
            { -1, -1, -1, -1, 11, 3, 0, 4, 10, 2, 1, 5, 9, 8, 7, 6 },
            { 9, 10, 11, -1, 8, 2, 3, -1, 7, 1, 0, -1, 6, 5, 4, -1 },
            { 6, 7, 8, 9, 5, 1, 2, 10, 4, 0, 3, 11, -1, -1, -1, -1 },
            { -1, 4, 5, 6, -1, 0, 1, 7, -1, 3, 2, 8, -1, 11, 10, 9 } };
        permutation = permuteBoundary[bIndex];
        level.gatherQuadRegularBoundaryPatchPoints(pn.faceIndex, patchVerts, bIndex);
    } else if (bType == 2) {
        // Expand corner patch vertices and rotate to restore correct orientation.
        static int const permuteCorner[4][16] = {
            { -1, -1, -1, -1, -1, 0, 1, 4, -1, 3, 2, 5, -1, 8, 7, 6 },
            { -1, -1, -1, -1, 8, 3, 0, -1, 7, 2, 1, -1, 6, 5, 4, -1 },
            { 6, 7, 8, -1, 5, 2, 3, -1, 4, 1, 0, -1, -1, -1, -1, -1 },
            { -1, 4, 5, 6, -1, 1, 2, 7, -1, 0, 3, 8, -1, -1, -1, -1 } };
        permutation = permuteCorner[bIndex];
        level.gatherQuadRegularCornerPatchPoints(pn.faceIndex, patchVerts, bIndex);
    } else {
        assert(bType <= 2);
    }

    short u, v;
    bool nonquad = computeSubPatchDomain(pn.levelIndex, pn.faceIndex, &u, &v);

    offsetAndPermuteIndices(patchVerts,
        16, levelVertOffset, permutation, supportsPtr + pn.firstSupport);

    // copy to buffers
    treePtr += pn.treeOffset;

    ((NodeDescriptor *)treePtr)->SetPatch(Characteristic::NODE_REGULAR,
        nonquad, singleCrease, pn.levelIndex, boundaryMask, u, v);

    treePtr[1] = pn.firstSupport;

    if (singleCrease) {
        *((float *)&treePtr[2]) = sharpness;
    }
}


void
CharacteristicBuilder::populateEndCapNode(
    ProtoNode const & pn, int * treePtr, Index * supportsPtr) const {

    // use the end-cap builder to generate support stencils for the end-cap
    // patches (bilinear, b-spline, gregory...)
    Vtr::internal::Level const & level = _refiner.getLevel((int)pn.levelIndex);

    Index levelVertOffset = _levelVertOffsets[pn.levelIndex];

    ConstIndexArray cvs =
        _endcapBuilder->GatherPatchPoints(level,
            pn.faceIndex, levelVertOffset, useInfSharpPatches(), useLegacySharpCornerPatches());

    memcpy(supportsPtr + pn.firstSupport, cvs.begin(), cvs.size()*sizeof(Index));

    short u, v;
    bool nonquad = computeSubPatchDomain(pn.levelIndex, pn.faceIndex, &u, &v);

    treePtr += pn.treeOffset;

    ((NodeDescriptor *)treePtr)->SetPatch(Characteristic::NODE_END, nonquad, false, pn.levelIndex, 0, u, v);

    treePtr[1] = pn.firstSupport;
}

void
CharacteristicBuilder::populateTerminalNode(
    ProtoNode const & pn, int * treePtr, Index * supportsPtr) const {

    // xxxx manuelk right now terminal nodes are recursive : paper suggests
    // a single node packing 25 * level supports. Considering memory gains
    // insignificant against code readability - if we want to change this,
    // we will have to change the descriptor layout.

    assert(pn.evIndex!=INDEX_INVALID);

    int childLevelIndex = pn.levelIndex + 1;

    Vtr::internal::Level const & childLevel = _refiner.getLevel(childLevelIndex);

    Index * firstSupport = supportsPtr + pn.firstSupport;

    for (int i=0; i<(int)pn.numChildren; ++i) {

        // gather support indices for the 3 regular sub-patches

        ProtoNode const & child = getNodeChild(pn, i);

        if (pn.evIndex!=i) {

            // regular patch children nodes should have been de-activated when
            // this node was identified as Terminal
            assert(child.active==false);

            // collect the support stencil indices
            Index localVerts[16], patchVerts[16];
            childLevel.gatherQuadRegularInteriorPatchPoints(child.faceIndex, localVerts, 0);

            static int const permuteRegular[16] = { 5, 6, 7, 8, 4, 0, 1, 9, 15, 3, 2, 10, 14, 13, 12, 11 };
            int levelVertOffset = _levelVertOffsets[childLevelIndex];
            offsetAndPermuteIndices(localVerts, 16, levelVertOffset, permuteRegular, patchVerts);

            // merge non-overlapping indices into a 5x5 array
                   if (i == ((pn.evIndex+2)%4)) {
                copyDiagonalIndices(pn.evIndex, patchVerts, firstSupport);
            } else if (i == (5-pn.evIndex)%4) {
                copyRowIndices(pn.evIndex, patchVerts, firstSupport);
            } else if (i == (3-pn.evIndex)%4) {
                copyColIndices(pn.evIndex, patchVerts, firstSupport);
            } else {
                assert(0);
            }
        }
        // not a recursive traversal : the xordinary child proto-node will be
        // handled when its proto-node is processed
    }

    if (useDynamicIsolation()) {

        // collect the support points for the dynamic end-cap covering this face
        int levelIndex = (int)pn.levelIndex,
            levelVertOffset = _levelVertOffsets[levelIndex];

        Vtr::internal::Level const & level = _refiner.getLevel(levelIndex);

        ConstIndexArray cvs =
            _endcapBuilder->GatherPatchPoints(level,
                pn.faceIndex, levelVertOffset, useInfSharpPatches(), useLegacySharpCornerPatches());

        memcpy(supportsPtr + pn.firstSupport + 25, cvs.begin(), cvs.size()*sizeof(Index));
    }

    // set corner support index for the EV to INVALID
    static int emptyIndices[4] = {0, 4, 24, 20 };
    firstSupport[emptyIndices[pn.evIndex]] = INDEX_INVALID;

    short u, v;
    bool nonquad = computeSubPatchDomain(childLevelIndex, getNodeChild(pn, 0).faceIndex, &u, &v);

    treePtr += pn.treeOffset;

    ((NodeDescriptor *)treePtr)->SetTerminal(nonquad, pn.levelIndex, permuteWinding(pn.evIndex), u, v);

    treePtr[1] = pn.firstSupport;
    treePtr[2] = getNodeChild(pn, pn.evIndex).treeOffset;
}

void
CharacteristicBuilder::populateRecursiveNode(
    ProtoNode const & pn, int * treePtr, Index * supportsPtr) const {

    treePtr += pn.treeOffset;

    if (useDynamicIsolation()) {

        int levelIndex = (int)pn.levelIndex,
            levelVertOffset = _levelVertOffsets[levelIndex];

        Vtr::internal::Level const & level = _refiner.getLevel(levelIndex);

        ConstIndexArray cvs =
            _endcapBuilder->GatherPatchPoints(level,
                pn.faceIndex, levelVertOffset, useInfSharpPatches(), useLegacySharpCornerPatches());

        memcpy(supportsPtr + pn.firstSupport, cvs.begin(), cvs.size()*sizeof(Index));

        short u, v;
        bool nonquad = computeSubPatchDomain(pn.levelIndex, pn.faceIndex, &u, &v);

        ((NodeDescriptor *)treePtr)->SetRecursive(nonquad, pn.levelIndex, u, v);

        treePtr[1] = pn.firstSupport;
    } else {

        ((NodeDescriptor *)treePtr)->SetRecursive(0, pn.levelIndex, 0, 0);

        treePtr[1] = INDEX_INVALID;
    }

    treePtr += 2;
    assert((int)pn.numChildren==4);
    for (int i=0; i<(int)pn.numChildren; ++i) {
        treePtr[permuteWinding(i)] = getNodeChild(pn, i).treeOffset;
    }

}

void
CharacteristicBuilder::populateNodes(int * treePtr, Index * supportsPtr) const {

    // populate tree & collect support stencil indices
    for (short level=0; level<_refiner.GetNumLevels(); ++level) {

        for (int pnIndex=0; pnIndex<(int)_nodeStore[level].size(); ++pnIndex) {

            ProtoNode const & pn = _nodeStore[level][pnIndex];

            if (!pn.active) {
                continue;
            }

            switch ((Characteristic::NodeType)pn.nodeType) {
                case Characteristic::NODE_REGULAR:
                    populateRegularNode(pn, treePtr, supportsPtr); break;
                case Characteristic::NODE_END:
                    populateEndCapNode(pn, treePtr, supportsPtr); break;
                case Characteristic::NODE_TERMINAL:
                    populateTerminalNode(pn, treePtr, supportsPtr); break;
                case Characteristic::NODE_RECURSIVE:
                    populateRecursiveNode(pn, treePtr, supportsPtr); break;
            }
        }
    }
}

Characteristic const *
CharacteristicBuilder::Create(
    Index levelIndex, Index faceIndex, Neighborhood const * neighborhood) {

    assert(neighborhood);

    // make sure the proto-node store is empty for each new characteristic
    resetNodeStore();

    // traverse topology recursively & collect proto-nodes
    identifyNode(levelIndex, faceIndex);

    if (useTerminalNodes()) {
        // prune recursive nodes that match terminal node configuration
        identifyTerminalNodes();
    }

    bool nonquad = levelIndex!=0;
    Characteristic * ch =
        new Characteristic(_charmap, neighborhood->GetNumVertices(), nonquad);

    // compute serial offsets for the Characteristic::Nodes tree
    int treeSize = 0, numSupportsTotal = 0;
    computeNodeOffsets(&treeSize, ch->_numSupports, &numSupportsTotal);


    BuildContext * context = new BuildContext;
    context->characteristic = ch;
    context->neighborhood = neighborhood;
    context->levelIndex = levelIndex;
    context->faceIndex = faceIndex;
    context->numSupportsTotal = numSupportsTotal;

    // populate the characteristic's tree & collect support indices
    ch->_tree.resize(treeSize);
    int * treePtr = &ch->_tree[0];

    context->supportIndices.resize(numSupportsTotal, INDEX_INVALID);
    Index * supportsPtr = &context->supportIndices[0];

    populateNodes(treePtr, supportsPtr);

    // save the context for the "finalize" step
    _buildContexts.push_back(context);

    return ch;
}

int
CharacteristicBuilder::FinalizeSupportStencils() {

    // XXXX manuelk : need to switch this for a code path that only computes
    // the stencils needed for the neighborhoods not yet in the map instead of
    // factorizing the entire mesh, including all redundant topologies

    StencilTable const * supportStencils =
        generateStencilTable(_refiner, *_endcapBuilder);
    assert(supportStencils);

    int numContexts = (int)_buildContexts.size(),
        numMaxSupports = 0;

    for (int contextIndex=0; contextIndex<numContexts; ++contextIndex) {

        BuildContext const * context = _buildContexts[contextIndex];

        Characteristic * ch = context->characteristic;

        // count the total number of influence weights & indices for the supports
        int numSupports = context->numSupportsTotal,
            numWeights = 0;
        for (int i=0; i<numSupports; ++i) {
            int stencilIndex = context->supportIndices[i];
            numWeights += stencilIndex!=INDEX_INVALID ?
                supportStencils->GetSizes()[stencilIndex] : 0;
        }

        // generate the support stencil weights
        ch->_sizes.resize(numSupports);
        ch->_offsets.resize(numSupports);
        ch->_indices.resize(numWeights);
        ch->_weights.resize(numWeights);

        Neighborhood const * neighborhood = context->neighborhood;

        for (int i=0, offset=0; i<numSupports; ++i) {

            int stencilIndex = context->supportIndices[i],
                stencilSize = 0;
            if (stencilIndex==INDEX_INVALID) {
                // XXXX manuelk we could probably skip those if we adjust
                // offsets computations accordingly
                ch->_sizes[i] = stencilSize;
                ch->_offsets[i] = offset;
            } else {

                Stencil stencil = supportStencils->GetStencil(stencilIndex);
                assert(stencil.GetSize()>0);

                stencilSize = stencil.GetSize();
                ch->_sizes[i] = stencilSize;
                ch->_offsets[i] = offset;
                for (int k=0; k<stencilSize; ++k) {

                    Index index = stencil.GetVertexIndices()[k];

if (neighborhood->Remap(index)==(LocalIndex)INDEX_INVALID) {
     printf("index=%d\n", index);
     neighborhood->Print();
     fflush(stdout);
}
                    assert(index!=INDEX_INVALID &&
                        neighborhood->Remap(index)!=(LocalIndex)INDEX_INVALID);
                    ch->_indices[offset+k] = neighborhood->Remap(index);
                    ch->_weights[offset+k] = stencil.GetWeights()[k];
                }
                offset += stencilSize;
            }
        }
        delete context;

        numMaxSupports = std::max(numSupports, numMaxSupports);
    }
    _buildContexts.clear();
    return numMaxSupports;
}

} // end namespace internal
} // end namespace Far
} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

