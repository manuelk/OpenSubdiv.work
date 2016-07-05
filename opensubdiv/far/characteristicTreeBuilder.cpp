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

#include "../far/characteristicTreeBuilder.h"
#include "../far/characteristicMap.h"
#include "../far/endCapBSplineBasisPatchFactory.h"
#include "../far/endCapGregoryBasisPatchFactory.h"
#include "../far/patchFaceTag.h"
#include "../far/topologyRefinerFactory.h"
#include "../vtr/refinement.h"

#include <cassert>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

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
                assert(0);
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

    StencilTable const * FinalizeStencils() {
        if (endcapStencils && (endcapStencils->GetNumStencils() > 0)) {
            endcapStencils->finalize();
        } else {
            delete endcapStencils;
            endcapStencils = nullptr;
        }
        return endcapStencils;
    }

    StencilTable const * FinalizeVaryingStencils() {
        if (endcapVaryingStencils && (endcapVaryingStencils->GetNumStencils() > 0)) {
            endcapVaryingStencils->finalize();
        } else {
            delete endcapVaryingStencils;
            endcapVaryingStencils = nullptr;
        }
        return endcapVaryingStencils;
    }

    EndCapType type;

    union {
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

// rowIndex
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

// colIndex
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

typedef Characteristic::NodeDescriptor NodeDescriptor;

bool
CharacteristicTreeBuilder::computeSubPatchDomain(
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

        Vtr::Index parentFaceIndex    = refinement.getChildFaceParentFace(faceIndex);
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

int
CharacteristicTreeBuilder::writeRegularNode(
    int levelIndex, int faceIndex, uint8_t * data) const {

    PatchFaceTag const & patchTag = _levelPatchTags[levelIndex][faceIndex];

    int dataSize = sizeof(NodeDescriptor) + 16 * sizeof(int) + (patchTag.isSingleCrease ? sizeof(float) : 0);

    if (data) {

        Vtr::internal::Level const & level = _refiner.getLevel(levelIndex);

        Index patchVerts[16];

        int bIndex = patchTag.boundaryIndex,
            boundaryMask = patchTag.boundaryMask,
            transitionMask = patchTag.transitionMask,
            levelVertOffset = _levelVertOffsets[levelIndex];

        int const * permutation = 0;
        bool singleCrease = false;
        float sharpness = 0.f;

        if (patchTag.boundaryCount == 0) {
            static int const permuteRegular[16] = { 5, 6, 7, 8, 4, 0, 1, 9, 15, 3, 2, 10, 14, 13, 12, 11 };
            permutation = permuteRegular;
            level.gatherQuadRegularInteriorPatchPoints(faceIndex, patchVerts, 0);
            if (patchTag.isSingleCrease) {
                int maxIsolation = _charmap.GetOptions().maxIsolationLevel;
                singleCrease = true;
                boundaryMask = (1<<bIndex);
                sharpness = level.getEdgeSharpness((level.getFaceEdges(faceIndex)[bIndex]));
                sharpness = std::min(sharpness, (float)(maxIsolation-levelIndex));
            }
        } else if (patchTag.boundaryCount == 1) {
            // Expand boundary patch vertices and rotate to restore correct orientation.
            static int const permuteBoundary[4][16] = {
                { -1, -1, -1, -1, 11, 3, 0, 4, 10, 2, 1, 5, 9, 8, 7, 6 },
                { 9, 10, 11, -1, 8, 2, 3, -1, 7, 1, 0, -1, 6, 5, 4, -1 },
                { 6, 7, 8, 9, 5, 1, 2, 10, 4, 0, 3, 11, -1, -1, -1, -1 },
                { -1, 4, 5, 6, -1, 0, 1, 7, -1, 3, 2, 8, -1, 11, 10, 9 } };
            permutation = permuteBoundary[bIndex];
            level.gatherQuadRegularBoundaryPatchPoints(faceIndex, patchVerts, bIndex);
        } else if (patchTag.boundaryCount == 2) {
            // Expand corner patch vertices and rotate to restore correct orientation.
            static int const permuteCorner[4][16] = {
                { -1, -1, -1, -1, -1, 0, 1, 4, -1, 3, 2, 5, -1, 8, 7, 6 },
                { -1, -1, -1, -1, 8, 3, 0, -1, 7, 2, 1, -1, 6, 5, 4, -1 },
                { 6, 7, 8, -1, 5, 2, 3, -1, 4, 1, 0, -1, -1, -1, -1, -1 },
                { -1, 4, 5, 6, -1, 1, 2, 7, -1, 0, 3, 8, -1, -1, -1, -1 } };
            permutation = permuteCorner[bIndex];
            level.gatherQuadRegularCornerPatchPoints(faceIndex, patchVerts, bIndex);
        } else {
            assert(patchTag.boundaryCount <= 2);
        }

        short u, v;
        bool nonquad = computeSubPatchDomain(levelIndex, faceIndex, &u, &v);

        // copy to buffer
        ((NodeDescriptor *)data)->SetPatch(Characteristic::NODE_REGULAR,
            nonquad, singleCrease, levelIndex, boundaryMask, u, v) ;
        data += sizeof(NodeDescriptor);

        if (patchTag.isSingleCrease) {
            *(float *)data = sharpness;
            data += sizeof(float);
        }

        offsetAndPermuteIndices(patchVerts, 16, levelVertOffset, permutation, (Index *)data);
    }
    return dataSize;
}

int
CharacteristicTreeBuilder::writeEndCapNode(
    int levelIndex, int faceIndex, uint8_t * data) const {

    assert(_endcapBuilder);
    EndCapType type = _endcapBuilder->type;

    int dataSize = sizeof(NodeDescriptor);

    switch (type) {
        case ENDCAP_BSPLINE_BASIS:
            dataSize += 16 * sizeof(Index);
            break;
        case ENDCAP_GREGORY_BASIS:
            dataSize += 20 * sizeof(Index);
            break;
        default:
            assert(0);
    }

    if (data) {

        Vtr::internal::Level const & level = _refiner.getLevel(levelIndex);

        PatchFaceTag const * levelPatchTags = _levelPatchTags[levelIndex];

        Index levelVertOffset = _levelVertOffsets[levelIndex];

        ConstIndexArray cvs;
        switch (type) {
            case ENDCAP_BILINEAR_BASIS:
                assert(0);
                break;
            case ENDCAP_BSPLINE_BASIS:
                cvs = _endcapBuilder->bsplineBasis->GetPatchPoints(
                    &level, faceIndex, levelPatchTags, levelVertOffset);
                break;
            case ENDCAP_GREGORY_BASIS:
                cvs = _endcapBuilder->gregoryBasis->GetPatchPoints(
                    &level, faceIndex, levelPatchTags, levelVertOffset);
                break;
            default:
                assert(0);
        }

        // copy to buffer

        short u, v;
        bool nonquad = computeSubPatchDomain(levelIndex, faceIndex, &u, &v);

        // copy to buffer
        ((NodeDescriptor *)data)->SetPatch(Characteristic::NODE_END,
            nonquad, false, levelIndex, 0, u, v);
        data += sizeof(NodeDescriptor);

        assert(sizeof(Index)==sizeof(int));
        memcpy(data, cvs.begin(), cvs.size() * sizeof(int));
    }
    return dataSize;
}

int
CharacteristicTreeBuilder::writeRecursiveNode(
    int levelIndex, int faceIndex, int offset, uint8_t * data) const {

    int dataSize = sizeof(NodeDescriptor) + 4 * sizeof(int);

    if (data) {
        ((NodeDescriptor *)data)->SetRecursive(levelIndex);

        int * childrenOffsets = (int *)(data + sizeof(NodeDescriptor));

        ConstIndexArray children =
            _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);

        for (int i=0; i<children.size(); ++i) {
            // save the offset in the nodes array where each child is stored
            int childOffset = offset + dataSize/sizeof(int);
            dataSize += writeNode(levelIndex+1, children[i], childOffset, data+dataSize);
            childrenOffsets[permuteWinding(i)] = childOffset;
        }
    } else {
        // traverse each child to accumulate data size
        ConstIndexArray children = _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);
        for (int i=0; i<children.size(); ++i) {
            dataSize += writeNode(levelIndex+1, children[i], 0, nullptr);
        }
    }
    return dataSize;
}

bool
CharacteristicTreeBuilder::nodeIsTerminal(
    int levelIndex, int faceIndex, int * evIndex) const {

    if (_charmap.GetOptions().useTerminalNode) {

        PatchFaceTag const * levelPatchTags = _levelPatchTags[levelIndex+1];

        ConstIndexArray children =
            _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);

        if (children.size()!=4) {
            return false;
        }

        int regular = 0, irregular = 0;
        for (int i=0; i<children.size(); ++i) {

            Index child = children[i];
            if (child==INDEX_INVALID) {
                return false;
            }

            PatchFaceTag const & patchTag = levelPatchTags[child];

            if (patchTag.isRegular) {
                assert(patchTag.hasPatch);
                ++regular;
            } else {

                // trivial rejection for boundaries or creases
                if ((patchTag.boundaryCount>0) || patchTag.isSingleCrease) {
                    return false;
                }

                // complete check
                Vtr::internal::Level const * level = &_refiner.getLevel(levelIndex);

                Vtr::ConstIndexArray fVerts = level->getFaceVertices(faceIndex);
                assert(fVerts.size() == 4);

                Vtr::internal::Level::VTag vt = level->getFaceCompositeVTag(fVerts);
                if (vt._semiSharp || vt._semiSharpEdges || vt._rule!=Sdc::Crease::RULE_SMOOTH) {
                    return false;
                }

                irregular = i;
            }
        }
        if (regular==3) {
            if (evIndex) {
                *evIndex = irregular;
            }
            return true;
        }
    }
    return false;
}

int
CharacteristicTreeBuilder::writeTerminalNode(
    int levelIndex, int faceIndex, int evIndex, int offset, uint8_t * data) const {

    int dataSize = sizeof(NodeDescriptor) + 1*sizeof(int) + 25*sizeof(int);

    if (data) {

        assert(evIndex!=INDEX_INVALID);

        int childLevelIndex = levelIndex + 1,
            childNodeOffset = 0,
            levelVertOffset = _levelVertOffsets[childLevelIndex];

        PatchFaceTag const * levelPatchTags =
            _levelPatchTags[childLevelIndex];

        Vtr::internal::Level const & childLevel =
            _refiner.getLevel(childLevelIndex);

        ConstIndexArray childFaceIndices =
            _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);

        // gather support indices for the 3 sub-patches
        Index * supportIndices = (int *)(data + sizeof(NodeDescriptor) + sizeof(int));
        for (int child=0; child<childFaceIndices.size(); ++child) {

            int childFaceIndex = childFaceIndices[child];

            PatchFaceTag const & patchTag =
                _levelPatchTags[childLevelIndex][childFaceIndex];

            if (evIndex!=child) {
                // child is a regular patch : get the supports
                Index localVerts[16], patchVerts[16];
                childLevel.gatherQuadRegularInteriorPatchPoints(childFaceIndex, localVerts, 0);
                static int const permuteRegular[16] = { 5, 6, 7, 8, 4, 0, 1, 9, 15, 3, 2, 10, 14, 13, 12, 11 };
                offsetAndPermuteIndices(localVerts, 16, levelVertOffset, permuteRegular, patchVerts);

                // copy non-overlapping indices into the node
                       if (child == ((evIndex+2)%4)) {
                    copyDiagonalIndices(evIndex, patchVerts, supportIndices);
                } else if (child == (5-evIndex)%4) {
                    copyRowIndices(evIndex, patchVerts, supportIndices);
                } else if (child == (3-evIndex)%4) {
                    copyColIndices(evIndex, patchVerts, supportIndices);
                } else {
                    assert(0);
                }
            } else {
                // child contains the EV
                assert(evIndex==child);

                // save the offset to the end-cap child node
                childNodeOffset = offset + dataSize / sizeof(int);

                if (patchTag.hasPatch) {
                    // we have reached the maximum isolation : end-cap node
                    dataSize += writeEndCapNode(childLevelIndex, childFaceIndex, data + dataSize);
                } else {
                    dataSize += writeTerminalNode(childLevelIndex,
                        childFaceIndex, evIndex, childNodeOffset, data+dataSize);
                }
            }
        }
        // set support index for the EV to INVALID
        static int emptyIndices[4] = {0, 4, 24, 20 };
        supportIndices[emptyIndices[evIndex]] = INDEX_INVALID;

        short u, v;
        bool nonquad = computeSubPatchDomain(childLevelIndex, childFaceIndices[0], &u, &v);

        ((NodeDescriptor *)data)->SetTerminal(nonquad, levelIndex, permuteWinding(evIndex), u, v);

        int * childNodePtr = (int *)(data + sizeof(NodeDescriptor));
        *childNodePtr = childNodeOffset;

    } else {
        // we don't need to recurse here : we know that there will always be
        // one terminal node + one end-cap node for each level of isolation left
        int endNodeSize = writeEndCapNode(0, 0, nullptr),
            termNodeSize = sizeof(NodeDescriptor) + 1*sizeof(int) + 25*sizeof(int);
        return dataSize + termNodeSize * (_refiner.GetMaxLevel()-levelIndex-1) + endNodeSize;
    }
    return dataSize;
}

int
CharacteristicTreeBuilder::writeNode(
    int levelIndex, int faceIndex, int offset, uint8_t * data) const {

    PatchFaceTag const & patchTag = _levelPatchTags[levelIndex][faceIndex];

    int dataSize = 0;

    if (patchTag.hasPatch) {
        if (patchTag.isRegular) {
            dataSize = writeRegularNode(levelIndex, faceIndex, data);
        } else {
            dataSize = writeEndCapNode(levelIndex, faceIndex, data);
        }
    } else {
        int evIndex = INDEX_INVALID;
        if (nodeIsTerminal(levelIndex, faceIndex, &evIndex)) {
            dataSize = writeTerminalNode(levelIndex, faceIndex, evIndex, offset, data);
        } else {
            dataSize = writeRecursiveNode(levelIndex, faceIndex, offset, data);
        }
    }
    return dataSize;
}

int
CharacteristicTreeBuilder::GetTreeSize(
    int levelIndex, int faceIndex) const {

    return writeNode(levelIndex, faceIndex, 0, nullptr) / sizeof(int);
}

void
CharacteristicTreeBuilder::WriteTree(
    int levelIndex, int faceIndex, int * treePtr) const {

    assert(treePtr);
    writeNode(levelIndex, faceIndex, 0, (uint8_t *)treePtr);
}

StencilTable const *
CharacteristicTreeBuilder::FinalizeStencils() {
    assert(_endcapBuilder);
    return _endcapBuilder->FinalizeStencils();
}

StencilTable const *
CharacteristicTreeBuilder::FinalizeVaryingStencils() {
    assert(_endcapBuilder);
    return _endcapBuilder->FinalizeVaryingStencils();
}

// constructor
CharacteristicTreeBuilder::CharacteristicTreeBuilder(
    TopologyRefiner const & refiner, CharacteristicMap const & charmap) :
        _refiner(refiner), _charmap(charmap) {

    CharacteristicMap::Options options = _charmap.GetOptions();

    // if the characteristic map is not empty, we want to append the new trees
    // at the end of the array, so we need to initialize the offset
    _treeOffset = _charmap.GetCharacteristicTreeSizeTotal();

    // identify patch types
    bool useSingleCrease = _charmap.GetOptions().useSingleCreasePatch;
    Far::PatchFaceTag::IdentifyAdaptivePatches(
        _refiner, _patchTags, _refiner.GetNumLevels(), useSingleCrease);

    _endcapBuilder = new EndCapBuilder(refiner, options.GetEndCapType());

    // gather starting offsets for patch tags & vertex indices for each
    // subdivision level
    int nlevels = _refiner.GetNumLevels(),
        levelFaceOffset = 0,
        levelVertOffset = 0;

    _levelPatchTags.resize(nlevels, 0);
    _levelVertOffsets.resize(nlevels,0);

    for (int i=0; i<nlevels; ++i) {

        TopologyLevel const & level = _refiner.GetLevel(i);

        _levelPatchTags[i] = & _patchTags[levelFaceOffset];
        _levelVertOffsets[i] = levelVertOffset;

        levelFaceOffset += level.GetNumFaces();
        levelVertOffset += level.GetNumVertices();
    }
}

CharacteristicTreeBuilder::~CharacteristicTreeBuilder() {
    delete _endcapBuilder;
}

} // end namespace Far
} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

