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

#include "../far/characteristicFactory.h"
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
           CharacteristicFactory::Options options);

    ~CharacteristicBuilder();

    void WriteCharacteristicTree(Characteristic * ch, int levelIndex, int faceIndex) const;

Characteristic * currentCh;

private:

    bool nodeIsTerminal(int levelIndex, int faceIndex) const;

    int writeNode(int leveIndex, int faceIndex, int offset, uint8_t * data) const;

    int writeRecursiveNode(int leveIndex, int faceIndex, int offset, uint8_t * data) const;

    int writeRegularNode(int leveIndex, int faceIndex, uint8_t * data) const;

    int writeEndNode(int leveIndex, int faceIndex, uint8_t * data) const;

    int writeTerminalNode(int leveIndex, int faceIndex, uint8_t * data) const;

private:

    //
    // General state
    //

    CharacteristicFactory::Options _options;

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
            CharacteristicFactory::Options options) :
                _options(options), _refiner(refiner), _patchTags(patchTags) {


    // create stencil tables for end caps w/ matching factory
    _endcapStencils = new StencilTable(0);
    _endcapVaryingStencils = new StencilTable(0);

    switch (_options.GetEndCapType()) {
        case CharacteristicFactory::Options::ENDCAP_BSPLINE_BASIS:
            _endCapBSplineBasis =
                new EndCapBSplineBasisPatchFactory(
                    refiner, _endcapStencils, _endcapVaryingStencils);
            break;
        case CharacteristicFactory::Options::ENDCAP_GREGORY_BASIS:
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
        case CharacteristicFactory::Options::ENDCAP_GREGORY_BASIS:
            delete _endCapGregoryBasis;
            break;
        case CharacteristicFactory::Options::ENDCAP_BSPLINE_BASIS:
            delete _endCapBSplineBasis;
            break;
        default:
            assert(0);
    }
}


typedef Characteristic::NodeDescriptor NodeDescriptor;

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

        // only single-crease patch has a sharpness.
        float sharpness = 0;

        if (patchTag.boundaryCount == 0) {
            static int const permuteRegular[16] = { 5, 6, 7, 8, 4, 0, 1, 9, 15, 3, 2, 10, 14, 13, 12, 11 };
            permutation = permuteRegular;
            level.gatherQuadRegularInteriorPatchPoints(faceIndex, patchVerts, 0);
            if (patchTag.isSingleCrease) {
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
        ((NodeDescriptor *)data)->Set(Characteristic::NODE_REGULAR, levelIndex, boundaryMask, transitionMask);
        data += sizeof(NodeDescriptor);

        if (patchTag.isSingleCrease) {
            *(float *)data = sharpness;
            data += sizeof(float);
        }

#if 0
        offsetAndPermuteIndices(patchVerts, 16, levelVertOffset, permutation, (Index *)data);
#else
        for (int i=0; i<16; ++i) {
            ((int *)data)[i] = 222;
        }
#endif
    }

    return dataSize;
}

int
CharacteristicBuilder::writeEndNode(
    int levelIndex, int faceIndex, uint8_t * data) const {

    int dataSize = sizeof(NodeDescriptor);

    switch (_options.endCapType) {
        case CharacteristicFactory::Options::ENDCAP_BSPLINE_BASIS:
        case CharacteristicFactory::Options::ENDCAP_GREGORY_BASIS:
            dataSize += 16 * sizeof(Index);
            break;
        default:
            assert(0);
    }

    if (data) {
#if 0
        Vtr::internal::Level const & level = _refiner.getLevel(levelIndex);

        PatchFaceTag const * levelPatchTags = _levelPatchTags[levelIndex];

        Index levelVertOffset = _levelVertOffsets[levelIndex];

        ConstIndexArray cvs;
        switch (_options.endCapType) {
                assert(0);
                break;
            case CharacteristicFactory::Options::ENDCAP_BSPLINE_BASIS:
                cvs = _endCapBSplineBasis->GetPatchPoints(&level, faceIndex, levelPatchTags, levelVertOffset);
                break;
            case CharacteristicFactory::Options::ENDCAP_GREGORY_BASIS:
                cvs = _endCapGregoryBasis->GetPatchPoints(&level, faceIndex, levelPatchTags, levelVertOffset);
                break;
            default:
                assert(0);
        }

        // copy to buffer
        ((NodeDescriptor *)data)->Set(Characteristic::NODE_END, levelIndex, 0, 0);
        data += sizeof(NodeDescriptor);

        assert(sizeof(Index)==sizeof(int));
        memcpy(data, cvs.begin(), cvs.size() * sizeof(int));
#else
        ((NodeDescriptor *)data)->Set(Characteristic::NODE_END, levelIndex, 0, 0);
        data += sizeof(NodeDescriptor);
        for (int i=0; i<16; ++i) {
            ((int *)data)[i] = 333;
        }
#endif
    }

    return dataSize;
}

static void printTree(char const * type, int offset, Characteristic const * ch) {
    int size = ch->treeSize/4;
    printf("%s (%d) ofs=%d [\n", type, size, offset);
    for (int i=0; i<size; ++i) {
        printf("%*d, ", 4, ch->tree[i]);
        if ((i+1)%30==0) printf("\n");
    }
    printf("]\n\n"); fflush(stdout);
}

int
CharacteristicBuilder::writeRecursiveNode(
    int levelIndex, int faceIndex, int offset, uint8_t * data) const {

    int dataSize = sizeof(NodeDescriptor) + 4 * sizeof(int);

    if (data) {

        ((NodeDescriptor *)data)->Set(Characteristic::NODE_RECURSIVE, levelIndex, 0, 0);

        int * childrenOffsets = (int *)(data + sizeof(NodeDescriptor));

        ConstIndexArray children = _refiner.GetLevel(levelIndex).GetFaceChildFaces(faceIndex);
        for (int i=0; i<children.size(); ++i) {

            int childOffset = offset + dataSize/sizeof(int);

            dataSize += writeNode(levelIndex+1, children[i], childOffset, data+dataSize);

            childrenOffsets[i] = childOffset;
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
    return false; // XXXX TODO
}

int
CharacteristicBuilder::writeNode(
    int levelIndex, int faceIndex, int offset, uint8_t * data) const {

    PatchFaceTag const & patchTag = _levelPatchTags[levelIndex][faceIndex];

    int dataSize = 0;

    if (patchTag.hasPatch) {
        if (patchTag.isRegular) {
            dataSize = writeRegularNode(levelIndex, faceIndex, data);
//if (data) printTree("Regular", offset, currentCh);
        } else {
            dataSize = writeEndNode(levelIndex, faceIndex, data);
//if (data) printTree("End", offset, currentCh);
        }
    } else {
        if (nodeIsTerminal(levelIndex, faceIndex)) {
            // XXX TODO
        } else {
            dataSize = writeRecursiveNode(levelIndex, faceIndex, offset, data);
//if (data) printTree("Recursive", offset, currentCh);
        }
    }
    return dataSize;
}


void
CharacteristicBuilder::WriteCharacteristicTree(
    Characteristic * ch, int levelIndex, int faceIndex) const {

    ch->treeSize = writeNode(levelIndex, faceIndex, 0, nullptr);

    ch->tree = new int[ch->treeSize/sizeof(int)];

    writeNode(levelIndex, faceIndex, 0, (uint8_t *)ch->tree);
}

static void
PrintCharacteristicTreeNode(Characteristic const & ch, int offset) {

    typedef Characteristic::NodeDescriptor Descriptor;

    if (offset==0) {
        printf("subgraph {\n");
    }
    int const * data = ch.tree + offset;

    Descriptor const & desc = *(Descriptor *)data;

    switch (desc.GetType()) {
        case Characteristic::NODE_REGULAR :
            printf("  %d [label=\"R\", shape=circle]\n", offset);
            break;

        case Characteristic::NODE_END :
            printf("  %d [label=\"E\", shape=circle, style=filled, color=darkorange]\n", offset);
            break;

        case Characteristic::NODE_RECURSIVE : {
                printf("  %d [label=\"I\", shape=circle, style=filled, color=dodgerblue]\n", offset);
                int const * childOffsets = data + 1;
                for (int i=0; i<4; ++i) {
                    PrintCharacteristicTreeNode(ch, childOffsets[i]);
                    printf("  %d->%d;\n", offset, childOffsets[i]);
                }
            } break;

        case Characteristic::NODE_TERMINAL :
            printf("  %d [shape=circle, label=\"T\"]", offset);

        default:
            assert(0);
    }
    if (offset==0) {
        printf("}\n");
    }
}

//
// Characteristic factory
//

Characteristic const *
CharacteristicFactory::Create(TopologyRefiner const & refiner,
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
    Characteristic * result = new Characteristic[nchars],
                   * ch = result;
    for (int face = 0; face < coarseLevel.GetNumFaces(); ++face) {

        ConstIndexArray children = coarseLevel.GetFaceChildFaces(face);
        if (children.size()==regFaceSize) {
//builder.currentCh = ch;
            builder.WriteCharacteristicTree(ch, 0, face);
            PrintCharacteristicTreeNode(*ch, 0);
            ++ch;
        } else {
            for (int child=0; child<children.size(); ++child) {
//builder.currentCh = ch;
                builder.WriteCharacteristicTree(ch, 1, children[child]);
                PrintCharacteristicTreeNode(*ch, 0);
                ++ch;
            }
        }

    }

    return result;
}


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

