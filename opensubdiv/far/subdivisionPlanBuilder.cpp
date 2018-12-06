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

#include "../far/subdivisionPlanBuilder.h"
#include "../far/neighborhood.h"
#include "../far/sparseMatrix.h"
#include "../far/stencilTable.h"
#include "../far/stencilTableFactory.h"
#include "../far/topologyMap.h"
#include "../far/topologyRefinerFactory.h"
#include "../vtr/array.h"
#include "../vtr/level.h"
#include "../vtr/fvarLevel.h"
#include "../vtr/refinement.h"
#include "../vtr/stackBuffer.h"

#include <cassert>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

using Vtr::internal::Refinement;
using Vtr::internal::Level;
using Vtr::internal::FVarLevel;
using Vtr::internal::StackBuffer;

namespace internal {

//
//  Helper functions:
//

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

static void
offsetAndPermuteIndices(Index const indices[], int count,
    Index offset, int const permutation[], Index result[]) {

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

inline PatchBuilder::BasisType
convertBasis(SubdivisionPlan::EndCapType type) {
    switch (type) {
        case SubdivisionPlan::ENDCAP_BILINEAR_BASIS : return PatchBuilder::BASIS_LINEAR;
        case SubdivisionPlan::ENDCAP_BSPLINE_BASIS : return PatchBuilder::BASIS_REGULAR;
        case SubdivisionPlan::ENDCAP_GREGORY_BASIS : return PatchBuilder::BASIS_GREGORY;
        default :
            return PatchBuilder::BASIS_UNSPECIFIED;
    }
}

//
// Endcap stencil helper
//

using Vtr::ConstArray;

typedef SubdivisionPlan::NodeDescriptor NodeDescriptor;


class SubdivisionPlanBuilder::LocalPointHelper {

public:

    LocalPointHelper(
        TopologyRefiner const & refiner,
            StencilTableReal<float> * stencilTable) :
                _numLocalPoints(0), 
                _localPointOffset(0), 
                _refiner(refiner), 
                _stencilTable(stencilTable) {

        // fix me XXXX
        int nexpected = 1024 * 1024;
        _stencilTable->reserve(nexpected, nexpected * 9);
    }

    ~LocalPointHelper() {
        delete _stencilTable;
    }

    int AppendLocalPatchPoints(int levelIndex, Index faceIndex,
                               SparseMatrix<float> const & matrix,
                               PatchDescriptor::Type       patchType,
                               Index const                 sourcePoints[],
                               int                         sourcePointOffset,
                               Index                       patchPoints[]) {


        int numPatchPoints  = matrix.GetNumRows(),
            firstNewLocalPoint = _localPointOffset + _numLocalPoints,
            nextNewLocalPoint  = firstNewLocalPoint,
            patchPointOffset = _refiner.GetNumVerticesTotal();

        appendLocalPointStencils(matrix, sourcePoints, sourcePointOffset);    
        for (int i = 0; i < numPatchPoints; ++i) {
            patchPoints[i] = patchPointOffset + nextNewLocalPoint ++;
        }

        int numNewLocalPoints = nextNewLocalPoint - firstNewLocalPoint;

        _numLocalPoints += numNewLocalPoints;

        return numNewLocalPoints;
    }

    StencilTableReal<float> const * Finalize() {
        _stencilTable->generateOffsets();
        return _stencilTable;
    }

private:

    void appendLocalPointStencils(SparseMatrix<float> const & conversionMatrix,
                                  Index const                 sourcePoints[],
                                  int                         sourcePointOffset) {

        //
        //  Resize the StencilTable members to accomodate all rows and elements from
        //  the given set of points represented by the matrix
        //
        int numNewStencils = conversionMatrix.GetNumRows(),
            numNewElements = conversionMatrix.GetNumElements();

        size_t numOldStencils = _stencilTable->_sizes.size(),
               numOldElements = _stencilTable->_indices.size();

        //  Assign the sizes for the new stencils:
        _stencilTable->_sizes.resize(numOldStencils + numNewStencils);

        int * newSizes = &_stencilTable->_sizes[numOldStencils];
        for (int i = 0; i < numNewStencils; ++i) {
            newSizes[i] = conversionMatrix.GetRowSize(i);
        }

        //  Assign remapped indices for the stencils:
        _stencilTable->_indices.resize(numOldElements + numNewElements);

        int const * mtxIndices = &conversionMatrix.GetColumns()[0];
        int * newIndices = &_stencilTable->_indices[numOldElements];

        for (int i = 0; i < numNewElements; ++i) {
            newIndices[i] = sourcePoints[mtxIndices[i]] + sourcePointOffset;
        }

        //  Copy the stencil weights direct from the matrix elements:
        _stencilTable->_weights.resize(numOldElements + numNewElements);

        float const * mtxWeights = &conversionMatrix.GetElements()[0];
        float * newWeights = &_stencilTable->_weights[numOldElements];

        std::memcpy(newWeights, mtxWeights, numNewElements * sizeof(float));
    }


private:

    TopologyRefiner const & _refiner;

    int _numLocalPoints,
        _localPointOffset;

    StencilTableReal<float> * _stencilTable;
};

//
// SubdivisionPlan builder implementation
//


SubdivisionPlanBuilder::SubdivisionPlanBuilder(TopologyRefiner const & refiner,
    TopologyMap const & topomap) :
        _refiner(refiner),
        _topomap(topomap),
        _patchBuilder(0) {

    PatchBuilder::Options options;
    options.regBasisType = PatchBuilder::BASIS_REGULAR;
    options.irregBasisType = convertBasis(getEndCapType());
    options.fillMissingBoundaryPoints = true;
    options.approxInfSharpWithSmooth = false;
    options.approxSmoothCornerWithSharp = false;

    _patchBuilder = PatchBuilder::Create(refiner, options);

    // allocate scratch memory
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

    // allocate helper to accumulate endcap stencil table
    _stencilHelper = new LocalPointHelper(refiner, new StencilTableReal<float>(0));
}

SubdivisionPlanBuilder::~SubdivisionPlanBuilder() {
    delete _patchBuilder;
}

SubdivisionPlan::EndCapType
SubdivisionPlanBuilder::getEndCapType() const {
    return _topomap.GetOptions().GetEndCapType();
}

short
SubdivisionPlanBuilder::getMaxIsolationLevel() const {
    return _refiner.GetAdaptiveOptions().isolationLevel;
}
bool
SubdivisionPlanBuilder::useTerminalNodes() const {
    return _topomap.GetOptions().useTerminalNode;
}
bool
SubdivisionPlanBuilder::useDynamicIsolation() const {
    return _topomap.GetOptions().useDynamicIsolation;
}
bool
SubdivisionPlanBuilder::useLegacySharpCornerPatches() const {
    return _topomap.GetOptions().generateLegacySharpCornerPatches;
}
bool
SubdivisionPlanBuilder::useSingleCreasePatches() const {
    return _refiner.GetAdaptiveOptions().useSingleCreasePatch;
}
bool
SubdivisionPlanBuilder::useInfSharpPatches() const {
    return _refiner.GetAdaptiveOptions().useInfSharpPatch;
}


// proto nodes

inline SubdivisionPlanBuilder::ProtoNode const &
SubdivisionPlanBuilder::getNodeChild(ProtoNode const & pn, short childIndex) const {
    return const_cast<SubdivisionPlanBuilder *>(this)->getNodeChild(pn, childIndex);
}

inline SubdivisionPlanBuilder::ProtoNode &
SubdivisionPlanBuilder::getNodeChild(ProtoNode const & pn, short childIndex) {
    return _nodeStore[pn.levelIndex+1][pn.children[childIndex]];
}

void
SubdivisionPlanBuilder::resetNodeStore() {
    for (short level=0; level<numLevelMax; ++level) {
        _nodeStore[level].clear();
    }
}

void SubdivisionPlanBuilder::setFaceTags(
    FaceTags & tags, int levelIndex, Index faceIndex) const {

    tags.clear();

    tags.hasPatch = _patchBuilder->IsFaceAPatch(levelIndex, faceIndex) && 
        _patchBuilder->IsFaceALeaf(levelIndex, faceIndex);

    if (!tags.hasPatch) {
        return;
    }

    tags.isRegular = _patchBuilder->IsPatchRegular(levelIndex, faceIndex);
    if (tags.isRegular) {

        tags.boundaryMask =
            _patchBuilder->GetRegularPatchBoundaryMask(levelIndex, faceIndex);

        if (useSingleCreasePatches() && (tags.boundaryMask==0)) {

            PatchBuilder::SingleCreaseInfo info;
            if (_patchBuilder->IsRegularSingleCreasePatch(levelIndex, faceIndex, info)) {

                float sharpness = std::min(info.creaseSharpness, float(getMaxIsolationLevel() - levelIndex));
                if (sharpness>0.f) {
                    tags.isSingleCrease = true;
                    tags.boundaryIndex = info.creaseEdgeInFace;
                    tags.sharpness = sharpness;
                }
            }
        }
    }
    tags.transitionMask = tags.isRegular ?
        _patchBuilder->GetTransitionMask(levelIndex, faceIndex) : 0;
}

int
SubdivisionPlanBuilder::identifyNode(int levelIndex, Index faceIndex) {

    ProtoNode node;

    node.active = true;
    node.levelIndex = levelIndex;
    node.faceIndex = faceIndex;

    setFaceTags(node.faceTags, levelIndex, faceIndex);

    if (node.faceTags.hasPatch) {
        node.nodeType = node.faceTags.isRegular ?
            SubdivisionPlan::NODE_REGULAR : SubdivisionPlan::NODE_END;
        node.numChildren = 0;
    } else {
        ConstIndexArray children =
            _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);
        for (int i=0; i<children.size(); ++i) {
            node.children[i] = identifyNode(levelIndex+1, children[i]);
        }
        node.nodeType = SubdivisionPlan::NODE_RECURSIVE;
        node.numChildren = (int)children.size();
    }

    _nodeStore[levelIndex].push_back(node);
    return (int)_nodeStore[levelIndex].size()-1;
}

bool
SubdivisionPlanBuilder::nodeIsTerminal(ProtoNode const & pn, int * evIndex) const {

    if (pn.numChildren!=4) {
        return false;
    }

    int regular = 0, irregular = 0;
    for (int i=0; i<(int)pn.numChildren; ++i) {

        ProtoNode const & child = getNodeChild(pn, i);
        if (child.faceTags.isRegular) {
            assert(child.faceTags.hasPatch);
            ++regular;
        } else {
            // trivial rejection for boundaries or creases
            if ((child.faceTags.boundaryCount>0) ||
                 child.faceTags.isSingleCrease) {
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

int
SubdivisionPlanBuilder::computeNumSupports(
    SubdivisionPlan::NodeType nodeType, bool useDynamicIsolation) const {

    typedef SubdivisionPlan::Node Node;

    if (nodeType == SubdivisionPlan::NODE_REGULAR) {
        return 16;
    } else if (nodeType == SubdivisionPlan::NODE_END) {
        return Node::getNumEndCapSupports(getEndCapType());
    } else if (nodeType == SubdivisionPlan::NODE_RECURSIVE) {
        return useDynamicIsolation ?
            Node::getNumEndCapSupports(getEndCapType()) : 0;
    } else {
        int nsupports = 0;
        if (nodeType == SubdivisionPlan::NODE_TERMINAL) {
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
SubdivisionPlanBuilder::computeNodeOffsets(
    int * treeSizeOut, short * numSupportsOut, int * numSupportsTotalOut) {

    typedef SubdivisionPlan::Node Node;

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

            SubdivisionPlan::NodeType nodeType =
                (SubdivisionPlan::NodeType)pn.nodeType;

            treeSize += Node::getNodeSize(
                nodeType, (bool)pn.faceTags.isSingleCrease);

            numSupports += computeNumSupports(
                nodeType, useDynamicIsolation());
        }
        numSupportsOut[level] = numSupports;
    }

    // pad the per-level number of supports lookup table with the
    // total number of supports.
    for (int level=_refiner.GetNumLevels(); level<11; ++level) {
        numSupportsOut[level] = numSupports;
    }

    *treeSizeOut = treeSize;
    *numSupportsTotalOut = numSupports;
}

void
SubdivisionPlanBuilder::identifyTerminalNodes() {

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
                pn.nodeType = SubdivisionPlan::NODE_TERMINAL;
            }
        }
    }
}

bool
SubdivisionPlanBuilder::computeSubPatchDomain(
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
SubdivisionPlanBuilder::gatherIrregularPatchPoints(
    int level, Index face, int offset, Index * supports) const {

    Vtr::internal::Level::VSpan cornerSpans[4];
    _patchBuilder->GetIrregularPatchCornerSpans(level, face, cornerSpans);

    SparseMatrix<float> matrix;
     _patchBuilder->GetIrregularPatchConversionMatrix(level, face, cornerSpans, matrix);

    int numSourcePoints = matrix.GetNumColumns();

    StackBuffer<Index, 64, true> sourcePoints(numSourcePoints);
    int npoints = _patchBuilder->GetIrregularPatchSourcePoints(level, face, cornerSpans, sourcePoints);

    _stencilHelper->AppendLocalPatchPoints(level, face, matrix, _patchBuilder->GetIrregularPatchType(), sourcePoints, offset, supports);
}

void
SubdivisionPlanBuilder::populateRegularNode(
    ProtoNode const & pn, int * treePtr, Index * supportsPtr) const {

    int level = pn.levelIndex,
        boundaryMask = pn.faceTags.boundaryMask;

    Index face = pn.faceIndex;

    Index patchVerts[16];
    _patchBuilder->GetRegularPatchPoints(level, face, boundaryMask, patchVerts);

    short u, v;
    bool nonquad = computeSubPatchDomain(level, face, &u, &v);

    // copy data to node buffers
    int offset = _levelVertOffsets[level];
    offsetAndPermuteIndices(patchVerts, 16, offset, 0, supportsPtr + pn.firstSupport);
    treePtr += pn.treeOffset;

    bool singleCrease = pn.faceTags.isSingleCrease;
    if (singleCrease)
        boundaryMask = 1 << pn.faceTags.boundaryIndex;

    ((NodeDescriptor *)treePtr)->SetPatch(
        SubdivisionPlan::NODE_REGULAR, nonquad, singleCrease, level, boundaryMask, u, v);

    treePtr[1] = pn.firstSupport;

    if (singleCrease) {
        *((float *)&treePtr[2]) = pn.faceTags.sharpness;
    }
}

void
SubdivisionPlanBuilder::populateEndCapNode(
    ProtoNode const & pn, int * treePtr, Index * supportsPtr) const {

    int level = pn.levelIndex,
        offset = _levelVertOffsets[level];

    Index face = pn.faceIndex;

    gatherIrregularPatchPoints(level, face, offset, supportsPtr + pn.firstSupport);

    short u, v;
    bool nonquad = computeSubPatchDomain(level, face, &u, &v);

    treePtr += pn.treeOffset;

    ((NodeDescriptor *)treePtr)->SetPatch(
        SubdivisionPlan::NODE_END, nonquad, false, level, 0, u, v);

    treePtr[1] = pn.firstSupport;
}

void
SubdivisionPlanBuilder::populateRecursiveNode(
    ProtoNode const & pn, int * treePtr, Index * supportsPtr) const {

    treePtr += pn.treeOffset;

    if (useDynamicIsolation()) {

        // generate end-cap patch for this face for dynamic LOD
        int level = pn.levelIndex,
            offset = _levelVertOffsets[level];

        Index face = pn.faceIndex;

        gatherIrregularPatchPoints(level, face, offset, supportsPtr + pn.firstSupport);

        short u, v;
        bool nonquad = computeSubPatchDomain(level, face, &u, &v);

        ((NodeDescriptor *)treePtr)->SetRecursive(nonquad, level, u, v);

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
SubdivisionPlanBuilder::populateTerminalNode(
    ProtoNode const & pn, int * treePtr, Index * supportsPtr) const {

    // xxxx manuelk right now terminal nodes are recursive : paper suggests
    // a single node packing 25 * level supports. Considering memory gains
    // insignificant against code readability - if we want to change this,
    // we will have to change the descriptor layout.

    assert(pn.evIndex!=INDEX_INVALID);

    int childLevelIndex = pn.levelIndex + 1;

    Index * firstSupport = supportsPtr + pn.firstSupport;

    for (int i=0; i<(int)pn.numChildren; ++i) {

        // gather support indices for the 3 regular sub-patches

        ProtoNode const & childNode = getNodeChild(pn, i);

        if (pn.evIndex!=i) {

            // regular patch children nodes should have been de-activated when
            // this node was identified as Terminal
            assert(childNode.active==false);

            // XXXX can we have a terminal node w/ boundaries ? non-0 boundary mask work ?
            Index localVerts[16], patchVerts[16];
            _patchBuilder->GetRegularPatchPoints(childNode.levelIndex, childNode.faceIndex, 0, localVerts);

            // copy data to node buffers
            int offset = _levelVertOffsets[childNode.levelIndex];
            offsetAndPermuteIndices(localVerts, 16, offset, 0, patchVerts);

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
        int offset = _levelVertOffsets[pn.levelIndex];
        gatherIrregularPatchPoints(pn.levelIndex, pn.faceIndex, offset, supportsPtr + pn.firstSupport + 25);
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
SubdivisionPlanBuilder::populateNodes(int * treePtr, Index * supportsPtr) const {

    // populate tree & collect support stencil indices
    for (short level=0; level<_refiner.GetNumLevels(); ++level) {

        for (int pnIndex=0; pnIndex<(int)_nodeStore[level].size(); ++pnIndex) {

            ProtoNode const & pn = _nodeStore[level][pnIndex];

            if (!pn.active) {
                continue;
            }

            switch ((SubdivisionPlan::NodeType)pn.nodeType) {
                case SubdivisionPlan::NODE_REGULAR:
                    populateRegularNode(pn, treePtr, supportsPtr); break;
                case SubdivisionPlan::NODE_END:
                    populateEndCapNode(pn, treePtr, supportsPtr); break;
                case SubdivisionPlan::NODE_TERMINAL:
                    populateTerminalNode(pn, treePtr, supportsPtr); break;
                case SubdivisionPlan::NODE_RECURSIVE:
                    populateRecursiveNode(pn, treePtr, supportsPtr); break;
            }
        }
    }
}

SubdivisionPlan *
SubdivisionPlanBuilder::Create(Neighborhood const * neighborhood,
    Index levelIndex, Index faceIndex, Index subfaceIndex) {

    assert(neighborhood);

    // make sure the proto-node store is empty for each new plan
    resetNodeStore();

    // traverse topology recursively & collect proto-nodes
    identifyNode(levelIndex, faceIndex);

    if (useTerminalNodes()) {
        // prune recursive nodes that match terminal node configuration
        identifyTerminalNodes();
    }

    bool nonquad = levelIndex!=0;
    SubdivisionPlan * plan =
        new SubdivisionPlan(
            _topomap, neighborhood->GetNumVertices(),
                neighborhood->GetValence(), subfaceIndex);

    // compute serial offsets for the SubdivisionPlan::Nodes tree
    int treeSize = 0, numSupportsTotal = 0;
    computeNodeOffsets(&treeSize, plan->_numSupports, &numSupportsTotal);

    BuildContext * context = new BuildContext;
    context->plan = plan;
    context->neighborhood = neighborhood;
    context->levelIndex = levelIndex;
    context->faceIndex = faceIndex;
    context->numSupportsTotal = numSupportsTotal;

    // populate the plan's tree & collect support indices
    plan->_tree.resize(treeSize);
    int * treePtr = &plan->_tree[0];

    context->supportIndices.resize(numSupportsTotal, INDEX_INVALID);
    Index * supportsPtr = &context->supportIndices[0];

    populateNodes(treePtr, supportsPtr);

    // save the context for the "finalize" step
    _buildContexts.push_back(context);

    return plan;
}


int
SubdivisionPlanBuilder::FinalizeSupportStencils() {

    StencilTableReal<float> const * supportStencils = 0;
    { // create stencil tables
        StencilTableFactory::Options options;
        options.generateOffsets = true;
        options.generateControlVerts = true;
        options.generateIntermediateLevels = true;
        options.factorizeIntermediateLevels = true;

        StencilTableReal<float> const * regularStencils = 
            StencilTableFactory::Create(_refiner, options);

        assert(regularStencils);

        StencilTableReal<float> const * localPointStencils = 
            _stencilHelper->Finalize();

        if (regularStencils &&
            localPointStencils && localPointStencils->GetNumStencils()>0) {

            supportStencils =
                StencilTableFactoryReal<float>::AppendLocalPointStencilTable(
                    _refiner, regularStencils, localPointStencils);

            delete localPointStencils;
            delete regularStencils;
        } else {
            supportStencils = regularStencils;
        }
    }

    // XXXX manuelk : need to switch this for a code path that only computes
    // the stencils needed for the neighborhoods not yet in the map instead of
    // factorizing the entire mesh, including all redundant topologies
    assert(supportStencils);

    int numContexts = (int)_buildContexts.size(),
        numMaxSupports = 0;

    for (int contextIndex=0; contextIndex<numContexts; ++contextIndex) {

        BuildContext const * context = _buildContexts[contextIndex];

        SubdivisionPlan * plan = context->plan;

        // count the total number of influence weights & indices for the supports
        int numSupports = context->numSupportsTotal,
            numWeights = 0;
        for (int i=0; i<numSupports; ++i) {
            int stencilIndex = context->supportIndices[i];
            numWeights += stencilIndex!=INDEX_INVALID ?
                supportStencils->GetSizes()[stencilIndex] : 0;
        }

        // generate the support stencil weights
        plan->_sizes.resize(numSupports);
        plan->_offsets.resize(numSupports);
        plan->_indices.resize(numWeights);
        plan->_weights.resize(numWeights);

        Neighborhood const * neighborhood = context->neighborhood;

        for (int i=0, offset=0; i<numSupports; ++i) {

            int stencilIndex = context->supportIndices[i],
                stencilSize = 0;
            if (stencilIndex==INDEX_INVALID) {
                // These plans do not render, but keeping them in the table
                // keeps face indices consistent with the control cage.
                plan->_sizes[i] = stencilSize;
                plan->_offsets[i] = offset;
            } else {

                Stencil stencil = supportStencils->GetStencil(stencilIndex);
                assert(stencil.GetSize()>0);

                stencilSize = stencil.GetSize();
                plan->_sizes[i] = stencilSize;
                plan->_offsets[i] = offset;
                for (int k=0; k<stencilSize; ++k) {

                    Index index = stencil.GetVertexIndices()[k];

// XXXX manuelk debug
if (neighborhood->Remap(index)==(LocalIndex)INDEX_INVALID) {
     printf("index=%d\n", index);
     neighborhood->Print();
     fflush(stdout);
}
                    assert(index!=INDEX_INVALID &&
                        neighborhood->Remap(index)!=(LocalIndex)INDEX_INVALID);
                    plan->_indices[offset+k] = neighborhood->Remap(index);
                    plan->_weights[offset+k] = stencil.GetWeights()[k];
                }
                offset += stencilSize;
            }
        }
        delete context;

        numMaxSupports = std::max(numSupports, numMaxSupports);
    }
    _buildContexts.clear();

    delete supportStencils;

    return numMaxSupports;
}

} // end namespace internal
} // end namespace Far
} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

