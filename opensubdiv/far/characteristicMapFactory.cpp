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
#include "../far/endCapBSplineBasisPatchFactory.h"
#include "../far/endCapGregoryBasisPatchFactory.h"
#include "../far/patchBasis.h"
#include "../far/patchFaceTag.h"
#include "../far/topologyRefinerFactory.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

//
//  Helper functions:
//

inline bool isSharpnessEqual(float s1, float s2) { return (s1 == s2); }

//  Indexing sharpnesses
static inline int
assignSharpnessIndex(float sharpness, std::vector<float> & sharpnessValues) {

    // linear search : we don't expect too many different sharpness values...
    for (int i=0; i<(int)sharpnessValues.size(); ++i) {
        if (isSharpnessEqual(sharpnessValues[i], sharpness)) {
            return i;
        }
    }
    sharpnessValues.push_back(sharpness);
    return (int)sharpnessValues.size()-1;
}

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

    bool nodeIsTerminal(int levelIndex, int faceIndex) const;

    int writeNode(int leveIndex, int faceIndex, int offset, uint8_t * data) const;

    int writeRecursiveNode(int leveIndex, int faceIndex, int offset, uint8_t * data) const;

    int writeRegularNode(int leveIndex, int faceIndex, uint8_t * data) const;

    int writeEndNode(int leveIndex, int faceIndex, uint8_t * data) const;

    int writeTerminalNode(int leveIndex, int faceIndex, uint8_t * data) const;

    Characteristic::NodeDescriptor computeNodeDescriptor(Characteristic::NodeType type,
        int levelIndex, Index faceIndex, int boundaryMask, int transitionMask, bool singleCrease) const;

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
        case CharacteristicMap::ENDCAP_BSPLINE_BASIS:
            _endCapBSplineBasis =
                new EndCapBSplineBasisPatchFactory(
                    refiner, _endcapStencils, _endcapVaryingStencils);
            break;
        case CharacteristicMap::ENDCAP_GREGORY_BASIS:
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
        case CharacteristicMap::ENDCAP_GREGORY_BASIS:
            delete _endCapGregoryBasis;
            break;
        case CharacteristicMap::ENDCAP_BSPLINE_BASIS:
            delete _endCapBSplineBasis;
            break;
        default:
            assert(0);
    }
}


typedef Characteristic::NodeDescriptor NodeDescriptor;

NodeDescriptor
CharacteristicBuilder::computeNodeDescriptor(Characteristic::NodeType type,
    int levelIndex, Index faceIndex, int boundaryMask, int transitionMask, bool singleCrease) const {

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
                case 0 :                     break;
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

    NodeDescriptor desc;
    desc.Set(type, nonquad, singleCrease, levelIndex, boundaryMask, transitionMask, u, v);
    return desc;
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

        // copy to buffer
        NodeDescriptor desc = computeNodeDescriptor(Characteristic::NODE_REGULAR, levelIndex, faceIndex, boundaryMask, transitionMask, singleCrease);

        *((NodeDescriptor *)data) = desc;
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
        case CharacteristicMap::ENDCAP_BSPLINE_BASIS:
            dataSize += 16 * sizeof(Index);
            break;
        case CharacteristicMap::ENDCAP_GREGORY_BASIS:
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
                assert(0);
                break;
            case CharacteristicMap::ENDCAP_BSPLINE_BASIS:
                cvs = _endCapBSplineBasis->GetPatchPoints(&level, faceIndex, levelPatchTags, levelVertOffset);
                break;
            case CharacteristicMap::ENDCAP_GREGORY_BASIS:
                cvs = _endCapGregoryBasis->GetPatchPoints(&level, faceIndex, levelPatchTags, levelVertOffset);
                break;
            default:
                assert(0);
        }

        // copy to buffer
        *(NodeDescriptor *)data = computeNodeDescriptor(Characteristic::NODE_END, levelIndex, faceIndex, 0, 0, false);
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
        ((NodeDescriptor *)data)->Set(Characteristic::NODE_RECURSIVE, levelIndex, false, false, 0, 0, 0, 0);

        int * childrenOffsets = (int *)(data + sizeof(NodeDescriptor));

        ConstIndexArray children = _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);

        for (int i=0; i<children.size(); ++i) {

            // permute from CCW to Z pattern to match bitwise ~= traversal ???
            static int const permute[] = { 0, 1, 3, 2 };

            int childOffset = offset + dataSize/sizeof(int);
            dataSize += writeNode(levelIndex+1, children[i], childOffset, data+dataSize);
            childrenOffsets[permute[i]] = childOffset;
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
CharacteristicBuilder::nodeIsTerminal(int levelIndex, int faceIndex) const {
    if (_options.useTerminalNodes) {
        PatchFaceTag const * levelPatchTags = _levelPatchTags[levelIndex];

        int irregular = 0;

        ConstIndexArray children = _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);

        for (int i=0; i<children.size(); ++i) {

            PatchFaceTag const & patchTag = levelPatchTags[faceIndex];

            if (!patchTag.hasPatch || (patchTag.boundaryCount>0) || patchTag.isSingleCrease) {
                ++irregular;
            }
            if (irregular>1) {
                return false;
            }
        }
        return true;
    }
    return false;
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
        if (nodeIsTerminal(levelIndex, faceIndex)) {
            // XXX TODO
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

#if 1
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


//
// Characteristic
//

unsigned short
Characteristic::NodeDescriptor::GetTransitionCount() const {
    // patches cannot have more than 2 boundaries : return -1 if more than 2 bits are set
    // 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
    // 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111
    static int masks[] = { 0,  1,  1,  2,  1,  2,  2,  3,
                           1,  2,  2,  3,  2,  3,  3,  4,  };
    return masks[GetTransitionMask()];
}

unsigned short
Characteristic::NodeDescriptor::GetBoundaryCount() const {

    // patches cannot have more than 2 boundaries : return -1 if more than 2 bits are set
    // 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
    // 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111
    static int masks[] = { 0,  1,  1,  2,  1,  2,  2, -1,
                           1,  2,  2, -1,  2, -1, -1, -1,  };
    return masks[GetBoundaryMask()];
}
int
Characteristic::Node::GetNumChildrenNodes() const {
    NodeDescriptor desc = this->GetDescriptor();
    switch (desc.GetType()) {
        case Characteristic::NODE_TERMINAL: return 1;
        case Characteristic::NODE_RECURSIVE: return 4;
        default:
            return 0;
    }
}

Characteristic::Node
Characteristic::Node::GetChildNode(int childIndex) const {

    NodeDescriptor desc = this->GetDescriptor();

    switch (desc.GetType()) {
        case Characteristic::NODE_TERMINAL:
            assert(childIndex=0);
            break;
        case Characteristic::NODE_RECURSIVE: {
                int const * offsetPtr = getNodeData() +
                    sizeof(NodeDescriptor)/sizeof(int) + childIndex;
                return Node(_characteristic, *offsetPtr);
            }
        default:
            assert(0);
    }
    return *this;
}

ConstIndexArray
Characteristic::Node::GetSupportIndices() const {

    int const * supportsPtr = getNodeData() + sizeof(NodeDescriptor)/sizeof(int);

    NodeDescriptor desc = this->GetDescriptor();
    switch (desc.GetType()) {
        case Characteristic::NODE_REGULAR : {
            if (desc.SingleCrease()) {
                supportsPtr += sizeof(float)/sizeof(int);
            }
            return ConstIndexArray(supportsPtr, 16);
        }
        case Characteristic::NODE_END: {
            CharacteristicMap::EndCapType endType =
                GetCharacteristic()->GetCharacteristicMap()->GetEndCapType();
            int nsupports = 0;
            switch (endType) {
                case CharacteristicMap::ENDCAP_BSPLINE_BASIS : nsupports = 16; break;
                case CharacteristicMap::ENDCAP_GREGORY_BASIS : nsupports = 20; break;
                default:
                    assert(0);
            }
            return ConstIndexArray(supportsPtr, nsupports);
        }
        case Characteristic::NODE_RECURSIVE:
        case Characteristic::NODE_TERMINAL:
        default:
            return ConstIndexArray(nullptr, 0);
    }
}

Characteristic::Node
Characteristic::Node::operator ++() {

    NodeDescriptor desc = this->GetDescriptor();

    int offset = this->GetTreeOffset() + sizeof(NodeDescriptor)/sizeof(int);

    switch (desc.GetType()) {
        case Characteristic::NODE_REGULAR : {
            if (desc.SingleCrease()) {
                offset += sizeof(float)/sizeof(int);
            }
            offset += 16;
        } break;
        case Characteristic::NODE_END: {
            CharacteristicMap::EndCapType endType =
                GetCharacteristic()->GetCharacteristicMap()->GetEndCapType();
            int nsupports = 0;
            switch (endType) {
                case CharacteristicMap::ENDCAP_BSPLINE_BASIS : nsupports = 16; break;
                case CharacteristicMap::ENDCAP_GREGORY_BASIS : nsupports = 20; break;
                default:
                    assert(0);
            }
            offset += nsupports;
        } break;
        case Characteristic::NODE_RECURSIVE:
            offset += 4;
            break;
        case Characteristic::NODE_TERMINAL:
        default:
            assert(0);
            break;
    }
    _treeOffset = offset;
    return *this;
}

Characteristic::Node
Characteristic::GetTreeNode(float s, float t) const {

    assert(_tree && sizeof(NodeDescriptor)==sizeof(int));

    // traverse the sub-patch tree to the (s,t) coordinates
    int offset = 0, corner = 0;
    NodeDescriptor desc = _tree[offset];
    while (desc.GetType()==NODE_RECURSIVE) {
        if (s>0.5f) { corner ^= 1; s = 1 - s; }
        if (t>0.5f) { corner ^= 2; t = 1 - t; }
        s *= 2.0f;
        t *= 2.0f;
        offset = _tree[offset + 1 + corner];
        desc = _tree[offset];
    }
    return Node(this, offset);
}

static float
getSingleCreasePatchSegmentParameter(PatchParam param, float s, float t) {

    unsigned short boundaryMask = param.GetBoundary();
    float f;
    if ((boundaryMask & 1) != 0) {
        f = 1 - t;
    } else if ((boundaryMask & 2) != 0) {
        f = s;
    } else if ((boundaryMask & 4) != 0) {
        f = t;
    } else if ((boundaryMask & 8) != 0) {
        f = 1 - s;
    }
    return f;
}

Characteristic::Node
Characteristic::EvaluateBasis(Node n, float s, float t,
    float wP[], float wDs[], float wDt[]) const {

    NodeDescriptor desc = n.GetDescriptor();

    PatchParam param;
    param.Set(/*face id*/ 0, desc.GetU(), desc.GetV(), desc.GetDepth(), desc.NonQuadRoot(),
        desc.GetBoundaryMask(), desc.GetTransitionMask());

    if (desc.GetType()==NODE_REGULAR) {

        internal::GetBSplineWeights(param, s, t, wP, wDs, wDt);

    } else if (desc.GetType()==NODE_END) {

        CharacteristicMap::EndCapType type =
            GetCharacteristicMap()->GetEndCapType();
        switch (type) {
            case CharacteristicMap::ENDCAP_BSPLINE_BASIS :
                internal::GetBSplineWeights(param, s, t, wP, wDs, wDt);
                break;
            case CharacteristicMap::ENDCAP_GREGORY_BASIS :
                internal::GetGregoryWeights(param, s, t, wP, wDs, wDt);
                break;
            default:
                assert(0);
        }
    } else {
        assert(0);
    }

    return n;
}

Characteristic::Node
Characteristic::EvaluateBasis(float s, float t,
    float wP[], float wDs[], float wDt[]) const {

    Node n = GetTreeNode(s, t);

    return EvaluateBasis(n, s, t, wP, wDs, wDt);
}

static void PrintCharacteristicsDigraph(Characteristic const * chars, int nchars);

//
// Characteristic factory
//

CharacteristicMap const *
CharacteristicMapFactory::Create(TopologyRefiner const & refiner,
    PatchFaceTagVector const & patchTags, Options options) {

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

    // Allocate & write the characteristics
    CharacteristicMap * charmap =
        new CharacteristicMap(options.GetEndCapType());
    charmap->_characteristics.resize(nchars);

    Characteristic * ch = &charmap->_characteristics[0];

    for (int face = 0; face < coarseLevel.GetNumFaces(); ++face) {


        ConstIndexArray children = coarseLevel.GetFaceChildFaces(face);

        if (children.size()==regFaceSize) {
            ch->_characteristicMap = charmap;
            builder.WriteCharacteristicTree(ch, 0, face);
            ++ch;
        } else {
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
    for (int i=0; i<cvs.size(); ++i) {
        if (i>0 && ((i%4)!=0))
            printf(" ");
        if ((i%4)==0)
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

        case Characteristic::NODE_TERMINAL :
            printf("  %zu [shape=circle, label=\"T", nodeID);
            if (showIndices) {
                PrintNodeIndices(node.GetSupportIndices());
            }
            printf("\"]");

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
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

