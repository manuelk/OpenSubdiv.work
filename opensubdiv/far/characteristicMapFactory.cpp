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

#include "../far/characteristicMapFactory.h"
#include "../far/characteristicMap.h"
#include "../far/endCapBSplineBasisPatchFactory.h"
#include "../far/endCapGregoryBasisPatchFactory.h"
#include "../far/patchFaceTag.h"
#include "../far/topologyRefinerFactory.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

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


//
// Builder
//

// A specialized builder for subdivision plan hierarchies
class CharacteristicBuilder {

public:

    CharacteristicBuilder(TopologyRefiner const & refiner,
        PatchFaceTagVector const & patchTags,
           CharacteristicMapFactory::Options options);

    ~CharacteristicBuilder();

    void WriteCharacteristicTree(Characteristic * ch, int levelIndex, int faceIndex) const;

    StencilTable const * FinalizeStencils();

    StencilTable const * FinalizeVaryingStencils();

private:

    bool nodeIsTerminal(int levelIndex, int faceIndex, int * evIndex=0) const;

    int writeTerminalNode(int leveIndex, int faceIndex, int evIndex, int offset, uint8_t * data) const;

    int writeRecursiveNode(int leveIndex, int faceIndex, int offset, uint8_t * data) const;

    int writeRegularNode(int leveIndex, int faceIndex, uint8_t * data) const;

    int writeEndNode(int leveIndex, int faceIndex, uint8_t * data) const;

    int writeNode(int leveIndex, int faceIndex, int offset, uint8_t * data) const;

    bool computeSubPatchDomain(int levelIndex, Index faceIndex, short * s, short * t) const;

private:

    //
    // General state
    //

    CharacteristicMapFactory::Options _options;

    TopologyRefiner const & _refiner;

    PatchFaceTagVector const & _patchTags;

    //
    // End-cap stencils
    //

    union {
        EndCapBSplineBasisPatchFactory * _endCapBSplineBasis;
        EndCapGregoryBasisPatchFactory * _endCapGregoryBasis;
    };

    StencilTable * _endcapStencils;
    StencilTable * _endcapVaryingStencils;

    //
    // Misc. subdivision level offsets
    //

    std::vector<PatchFaceTag const *> _levelPatchTags;
    std::vector<Index> _levelVertOffsets;
};

CharacteristicBuilder::CharacteristicBuilder(
    TopologyRefiner const & refiner,
        PatchFaceTagVector const & patchTags,
            CharacteristicMapFactory::Options options) :
                _options(options), _refiner(refiner), _patchTags(patchTags) {

    // create stencil tables for end caps w/ matching factory
    _endcapStencils = new StencilTable(0);
    _endcapVaryingStencils = new StencilTable(0);

    switch (_options.GetEndCapType()) {
        case ENDCAP_BSPLINE_BASIS:
            _endCapBSplineBasis =
                new EndCapBSplineBasisPatchFactory(
                    refiner, _endcapStencils, _endcapVaryingStencils);
            break;
        case ENDCAP_GREGORY_BASIS:
            _endCapGregoryBasis =
                new EndCapGregoryBasisPatchFactory(
                    refiner, _endcapStencils, _endcapVaryingStencils);
            break;
        default:
            break;
    }

    // gather starting offsets for patch tags & vertex indices for each
    // subdivision level
    int nlevels = _refiner.GetNumLevels();

    _levelPatchTags.resize(nlevels, 0);
    _levelVertOffsets.resize(nlevels,0);

    int levelFaceOffset = 0,
        levelVertOffset = 0;

    for (int i=0; i<nlevels; ++i) {

        TopologyLevel const & level = _refiner.GetLevel(i);

        _levelPatchTags[i] = & patchTags[levelFaceOffset];
        _levelVertOffsets[i] = levelVertOffset;

        levelFaceOffset += level.GetNumFaces();
        levelVertOffset += level.GetNumVertices();
    }
}

CharacteristicBuilder::~CharacteristicBuilder() {
    switch (_options.GetEndCapType()) {
        case ENDCAP_GREGORY_BASIS:
            delete _endCapGregoryBasis;
            break;
        case ENDCAP_BSPLINE_BASIS:
            delete _endCapBSplineBasis;
            break;
        default:
            assert(0);
    }
}


typedef Characteristic::NodeDescriptor NodeDescriptor;

bool
CharacteristicBuilder::computeSubPatchDomain(int levelIndex, Index faceIndex, short * s, short * t) const {

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
CharacteristicBuilder::writeRegularNode(
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
                singleCrease = true;
                boundaryMask = (1<<bIndex);
                sharpness = level.getEdgeSharpness((level.getFaceEdges(faceIndex)[bIndex]));
                sharpness = std::min(sharpness, (float)(_refiner.GetMaxLevel()-levelIndex));
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
            nonquad, singleCrease, levelIndex, boundaryMask, transitionMask, u, v) ;
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
CharacteristicBuilder::writeEndNode(
    int levelIndex, int faceIndex, uint8_t * data) const {

    int dataSize = sizeof(NodeDescriptor);

    switch (_options.endCapType) {
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
        switch (_options.endCapType) {
            case ENDCAP_BILINEAR_BASIS:
                assert(0);
                break;
            case ENDCAP_BSPLINE_BASIS:
                cvs = _endCapBSplineBasis->GetPatchPoints(
                    &level, faceIndex, levelPatchTags, levelVertOffset);
                break;
            case ENDCAP_GREGORY_BASIS:
                cvs = _endCapGregoryBasis->GetPatchPoints(
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
            nonquad, false, levelIndex, 0, 0, u, v);
        data += sizeof(NodeDescriptor);

        assert(sizeof(Index)==sizeof(int));
        memcpy(data, cvs.begin(), cvs.size() * sizeof(int));
    }
    return dataSize;
}

int
CharacteristicBuilder::writeRecursiveNode(
    int levelIndex, int faceIndex, int offset, uint8_t * data) const {

    int dataSize = sizeof(NodeDescriptor) + 4 * sizeof(int);

    if (data) {
        ((NodeDescriptor *)data)->SetRecursive(levelIndex);

        int * childrenOffsets = (int *)(data + sizeof(NodeDescriptor));

        ConstIndexArray children =
            _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);

        for (int i=0; i<children.size(); ++i) {

            // convert CCW winding to match bitwise ^= traversal
            static int const permuteWinding[] = { 0, 1, 3, 2 };

            int childOffset = offset + dataSize/sizeof(int);
            dataSize += writeNode(levelIndex+1, children[i], childOffset, data+dataSize);
            childrenOffsets[permuteWinding[i]] = childOffset;
        }
    } else {
        ConstIndexArray children = _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);
        for (int i=0; i<children.size(); ++i) {
            dataSize += writeNode(levelIndex+1, children[i], 0, nullptr);
        }
    }
    return dataSize;
}

bool
CharacteristicBuilder::nodeIsTerminal(
    int levelIndex, int faceIndex, int * evIndex) const {

    if (_options.useTerminalNodes) {

        PatchFaceTag const * levelPatchTags = _levelPatchTags[levelIndex+1];

        int irregular = 0;

        ConstIndexArray children =
            _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);
        assert(children.size()==4);

        for (int i=0; i<children.size(); ++i) {

            PatchFaceTag const & patchTag = levelPatchTags[children[i]];

            if ((!patchTag.hasPatch) ||
                (!patchTag.isRegular) ||
                (patchTag.boundaryCount>0) ||
                (patchTag.isSingleCrease)) {
                if (evIndex) {
                    *evIndex = i;
                }
                ++irregular;
            }
            if (irregular>1) {
                return false;
            }
        }
        if (irregular==1) {
            return true;
        }
    }
    return false;
}

// Copy indices from 16-wide bicubic basis into 25-wide terminal node.
// Extraordinary vertex ('X') left "undefined"
//
//      Diagonal               Row                  Column
// +----------------+   +---+------------+   +----------------+
// | X   .  .  .  . |   | X | o  o  o  o |   | X   .  .  .  . |
// |   +------------+   |   +------------+   +---+            |
// | . | o  o  o  o |   | .   .  .  .  . |   | o | .  .  .  . |
// | . | o  o  o  o |   | .   .  .  .  . |   | o | .  .  .  . |
// | . | o  o  o  o |   | .   .  .  .  . |   | o | .  .  .  . |
// | . | o  o  o  o |   | .   .  .  .  . |   | o | .  .  .  . |
// +---+------------+   +----------------+   +---+------------+
inline void
copyDiagonalIndices(int evIndex, Index const * src, Index * dst) {
    // copy 16 verts by rows
    int rowOffs = evIndex < 2 ? 5 : 0,
        colOffs = evIndex==1 || evIndex==2 ? 0 : 1;
    Index * rowPtr = dst + rowOffs + colOffs;
    for (int k=0; k<4; ++k, rowPtr+=5) {
        memcpy(rowPtr, &src[k*4], 4 * sizeof(Index));
    }
}

inline void
copyRowIndices(int rowIndex, Index const * src, Index * dst) {
    int rowOffs = rowIndex > 1 ? 1 : 0,
        colOffs = rowIndex==1 || rowIndex==2 ? 1 : 0;
    Index * rowPtr = dst + rowOffs * 20 + colOffs;
    memcpy(rowPtr, &src[rowOffs * 12], 4 * sizeof(Index));
}

inline void
copyColIndices(int colIndex, Index const * src, Index * dst) {
    int rowOffs = colIndex > 1 ? 1 : 0,
        colOffs = colIndex==1 || colIndex==2 ? 1 : 0;
    src += colOffs * 3;
    dst += rowOffs * 5 + colOffs * 4;
    for (int i=0; i<4; ++i, dst+=5, src+=4) {
        *dst = *src;
    }
}

int
CharacteristicBuilder::writeTerminalNode(
    int levelIndex, int faceIndex, int evIndex, int offset, uint8_t * data) const {

    int dataSize = sizeof(NodeDescriptor) + 1*sizeof(int) + 25*sizeof(int);

    if (data) {

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

            if (patchTag.hasPatch && patchTag.isRegular) {
                // child is a regular patch : get the supports
                Index localVerts[16], patchVerts[16];
                childLevel.gatherQuadRegularInteriorPatchPoints(childFaceIndex, localVerts, 0);
                static int const permuteRegular[16] = { 5, 6, 7, 8, 4, 0, 1, 9, 15, 3, 2, 10, 14, 13, 12, 11 };
                offsetAndPermuteIndices(localVerts, 16, levelVertOffset, permuteRegular, patchVerts);

                if (child == ((evIndex+2)%4)) {
                    copyDiagonalIndices(evIndex, patchVerts, supportIndices);
                } else {
                    int rowIdx = (5-evIndex)%4,
                        colIdx = (3-evIndex)%4;
                    if (child == rowIdx) {
                        copyRowIndices(rowIdx, patchVerts, supportIndices);
                    } else if (child==colIdx) {
                        copyColIndices(colIdx, patchVerts, supportIndices);
                    } else {
                        assert(0);
                    }
                }
            } else {
                // child contains the EV
                assert(evIndex==child);

                // save the offset to the end-cap child node
                childNodeOffset = offset + dataSize / sizeof(int);

                if (patchTag.hasPatch) {
                    // we have reached the maximum isolation : end-cap node
                    dataSize += writeEndNode(childLevelIndex, childFaceIndex, data + dataSize);
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

        // convert from CCW winding to ^ bitwise order (0, 1, 3, 2)
        static int const permuteWinding[] = { 0, 1, 3, 2 };

        ((NodeDescriptor *)data)->SetTerminal(nonquad, levelIndex, permuteWinding[evIndex], u, v);

        int * childNodePtr = (int *)(data + sizeof(NodeDescriptor));
        *childNodePtr = childNodeOffset;

    } else {
        // XXXX this shoudld be a static method on Node
        int endNodeSize = writeEndNode(0, 0, nullptr),
            termNodeSize = sizeof(NodeDescriptor) + 1*sizeof(int) + 25*sizeof(int);
        // we don't need to recurse here : we know that there will always be
        // one terminal node for each level of isolation left and one end-cap node
        return dataSize + termNodeSize * (_refiner.GetMaxLevel()-levelIndex-1) + endNodeSize;
    }

    return dataSize;
}

int
CharacteristicBuilder::writeNode(
    int levelIndex, int faceIndex, int offset, uint8_t * data) const {

    PatchFaceTag const & patchTag = _levelPatchTags[levelIndex][faceIndex];

    int dataSize = 0;

    if (patchTag.hasPatch) {
        if (patchTag.isRegular) {
            dataSize = writeRegularNode(levelIndex, faceIndex, data);
        } else {
            dataSize = writeEndNode(levelIndex, faceIndex, data);
        }
    } else {
        int evIndex = -1;
        if (nodeIsTerminal(levelIndex, faceIndex, &evIndex)) {
            dataSize = writeTerminalNode(levelIndex, faceIndex, evIndex, offset, data);
        } else {
            dataSize = writeRecursiveNode(levelIndex, faceIndex, offset, data);
        }
    }
    return dataSize;
}

void
CharacteristicBuilder::WriteCharacteristicTree(
    Characteristic * ch, int levelIndex, int faceIndex) const {

    int nbytes = writeNode(levelIndex, faceIndex, 0, nullptr);

    ch->_treeSize = nbytes / sizeof(int);
    ch->_tree = new int[ch->_treeSize];

#if 0
    for (int i=0; i<ch->_treeSize; ++i) {
        ch->_tree[i] = -1;
    }
#endif

    writeNode(levelIndex, faceIndex, 0, (uint8_t *)ch->_tree);
    //PrintCharacteristicTreeNode(ch->tree, 0, 0);
}

StencilTable const *
CharacteristicBuilder::FinalizeStencils() {
    if (_endcapStencils && (_endcapStencils->GetNumStencils() > 0)) {
        _endcapStencils->finalize();
    } else {
        delete _endcapStencils;
        _endcapStencils = nullptr;
    }
    return _endcapStencils;
}

StencilTable const *
CharacteristicBuilder::FinalizeVaryingStencils() {
    if (_endcapVaryingStencils && (_endcapVaryingStencils->GetNumStencils() > 0)) {
        _endcapVaryingStencils->finalize();
    } else {
        delete _endcapVaryingStencils;
        _endcapVaryingStencils = nullptr;
    }
    return _endcapVaryingStencils;
}


static void PrintCharacteristicsDigraph(Characteristic const * chars, int nchars);

//
// Characteristic factory
//

CharacteristicMap const *
CharacteristicMapFactory::Create(TopologyRefiner const & refiner,
    PatchFaceTagVector const & patchTags, Options options) {

    // XXXX we do not support those end-cap types yet
    if (options.GetEndCapType()==ENDCAP_BILINEAR_BASIS ||
        options.GetEndCapType()==ENDCAP_LEGACY_GREGORY) {
        return nullptr;
    }

    CharacteristicBuilder builder(refiner, patchTags, options);

    TopologyLevel const & coarseLevel = refiner.GetLevel(0);

    int regFaceSize =
        Sdc::SchemeTypeTraits::GetRegularFaceSize(refiner.GetSchemeType());

    int nfaces = coarseLevel.GetNumFaces(),
        nchars = 0;

    // Count the number of characteristics (non-quads have more than 1)
    for (int face = 0; face < nfaces; ++face) {
        ConstIndexArray fverts = coarseLevel.GetFaceVertices(face);
        nchars += fverts.size()==regFaceSize ? 1 : fverts.size();
    }

    CharacteristicMap * charmap =
        new CharacteristicMap(options.GetEndCapType());

    // Allocate & write the characteristics
    charmap->_characteristics.resize(nchars);
    Characteristic * ch = &charmap->_characteristics[0];

    for (int face = 0; face < coarseLevel.GetNumFaces(); ++face) {

        if (coarseLevel.IsFaceHole(face)) {
            continue;
        }

        ConstIndexArray verts = coarseLevel.GetFaceVertices(face);

        if (verts.size()==regFaceSize) {
            ch->_characteristicMap = charmap;
            builder.WriteCharacteristicTree(ch, 0, face);
            ++ch;
        } else {
            ConstIndexArray children = coarseLevel.GetFaceChildFaces(face);
            for (int child=0; child<children.size(); ++child) {
                ch->_characteristicMap = charmap;
                builder.WriteCharacteristicTree(ch, 1, children[child]);
                ++ch;
            }
        }
    }

    charmap->_localPointStencils = builder.FinalizeStencils();
    charmap->_localPointVaryingStencils = builder.FinalizeVaryingStencils();

    //PrintCharacteristicsDigraph(&charmap->_characteristics[0], nchars);

    return charmap;
}

//
// Debug functions
//

static void
PrintNodeIndices(ConstIndexArray cvs) {

    int stride = cvs.size()==16 ? 4 : 5;

    for (int i=0; i<cvs.size(); ++i) {
        if (i>0 && ((i%stride)!=0))
            printf(" ");
        if ((i%stride)==0)
            printf("\\n");
        printf("%*d", 4, cvs[i]);
    }
}

inline size_t
HashNodeID(int charIndex, Characteristic::Node node) {
    size_t hash = node.GetTreeOffset() + ((size_t)charIndex << 32);
    return hash;
}

static void
PrintCharacteristicTreeNode(Characteristic::Node node, int charIndex, bool showIndices=false) {

    typedef Characteristic::NodeDescriptor Descriptor;

    Descriptor const & desc = node.GetDescriptor();

    size_t nodeID = HashNodeID(charIndex, node);

    switch (desc.GetType()) {
        case Characteristic::NODE_REGULAR : {
                printf("  %zu [label=\"R\\n", nodeID);
                if (showIndices) {
                    PrintNodeIndices(node.GetSupportIndices());
                }
                printf("\", shape=box]\n");
            } break;

        case Characteristic::NODE_END : {
                printf("  %zu [label=\"E\\n", nodeID);
                if (showIndices) {
                    PrintNodeIndices(node.GetSupportIndices());
                }
                printf("\", shape=box, style=filled, color=darkorange]\n");
            } break;

        case Characteristic::NODE_RECURSIVE : {
                printf("  %zu [label=\"I\", shape=square, style=filled, color=dodgerblue]\n",nodeID );
                for (int i=0; i<4; ++i) {
                    Characteristic::Node child = node.GetChildNode(i);
                    PrintCharacteristicTreeNode(child, charIndex, showIndices);
                    printf("  %zu -> %zu [label=\"%d\"]\n", nodeID, HashNodeID(charIndex, child), i);
                }
            } break;

        case Characteristic::NODE_TERMINAL : {
            printf("  %zu [shape=box, style=filled, color=grey, label=\"T", nodeID);
            if (showIndices) {
                PrintNodeIndices(node.GetSupportIndices());
            }
            printf("\"]\n");
            Characteristic::Node child = node.GetChildNode();
            PrintCharacteristicTreeNode(child, charIndex, showIndices);
            printf("  %zu -> %zu\n", nodeID, HashNodeID(charIndex, child));
        } break;
        default:
            assert(0);
    }
}

static void
PrintCharacteristicsDigraph(Characteristic const * chars, int nchars) {

    printf("digraph {\n");
    for (int i=0; i<nchars; ++i) {

        Characteristic const & ch = chars[i];

        printf("subgraph {\n");
        PrintCharacteristicTreeNode(ch.GetTreeRootNode(), i, /*show indices*/true);
        printf("}\n");
    }
    printf("}\n");
    fflush(stdout);
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

