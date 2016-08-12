//
//   Copyright 2013 Nvidia
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

#include <far/endCapBSplineBasisPatchFactory.h>
#include <far/endCapGregoryBasisPatchFactory.h>
#include <far/patchFaceTag.h>
#include <far/topologyRefinerFactory.h>

#include "init_shapes.h"

#include <string>
#include <fstream>
#include <sstream>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {


enum NodeType {
    NODE_UNUSED=0,
    NODE_NGON,
    NODE_REGULAR,
    NODE_BOUNDARY,
    NODE_CORNER,
    NODE_RECURSIVE,
    NODE_LIMIT,
    NODE_SEMI_SHARP,
};

static char const * NodeTypeNames[] = {
    "UNUSED",
    "NGON",
    "REGULAR",
    "BOUNDARY",
    "CORNER",
    "RECURSIVE",
    "LIMIT",
    "SEMI_SHARP"
};

struct Node {

    Node() {
        memset(this, 0, sizeof(Node));
    }

    void Print() const;

    NodeType type;

    int sharpnessIndex; // single crease sharpness

    Index childFaceCount,
          children[4],
          controlVerts[16];
};

void Node::Print() const {
    printf("{ \"type\":\"%s\", \"children\":%d", NodeTypeNames[type], childFaceCount);

    printf(", \"children\":[");
    for (int j=0; j<4; ++j) {
        if (j>0) printf(", ");
        printf("%d", children[j]);
    }
    printf("] }");

    printf(", \"cvs\":[");
    for (int j=0; j<16; ++j) {
        if (j>0) printf(", ");
        printf("%d", controlVerts[j]);
    }
    printf("] }");
}

///
/// Bitfield layout :
///
///  Field0     | Bits | Content
///  -----------|:----:|------------------------------------------------------
///  offset     | 28   | the faceId of the patch
///  type       | 4    | type
///
///  Field1     | Bits | Content
///  -----------|:----:|------------------------------------------------------
///  transition | 4    | transition edge mask encoding
///  level      | 4    | the subdivision level of the node
///  nonquad    | 1    | whether the patch is the child of a non-quad face
///  boundary   | 4    | boundary edge mask encoding
///
struct NodeBits {

    void Set(int offset, unsigned short type, unsigned short depth, bool nonquad,
                 unsigned short boundary, unsigned short transition ) {
        field0 = (((unsigned int)offset) & 0xfffffff) |
                 ((type & 0xf) << 28);
        field1 = ((boundary & 0xf) << 9) |
                 ((nonquad ? 1:0) << 8) |
                 ((nonquad ? depth+1 : depth) << 4) |
                 (transition & 0xf);
    }


    /// \brief Resets everything to 0
    void Clear() { field0 = field1 = 0; }

    /// \brief Retuns the offset
    int GetOffset() const { return Index(field0 & 0xfffffff); }

    /// \brief Returns the transition edge encoding for the patch.
    unsigned short GetTransition() const { return (unsigned short)((field0 >> 28) & 0xf); }

    /// \brief Returns the boundary edge encoding for the patch.
    unsigned short GetBoundary() const { return (unsigned short)((field1 >> 8) & 0xf); }

    /// \brief True if the parent coarse face is a non-quad
    bool NonQuadRoot() const { return (field1 >> 4) & 0x1; }

    unsigned int field0:32,
                 field1:32;
};

struct Characteristic {

    int * tree,
          treeSize,
          treeRoot;
};

/// \brief A specialized builder for subdivision plan hierarchies
///
class NodeTreeBuilder {

public:

    struct Options {

        enum EndCapType {
            ENDCAP_NONE = 0,             ///< no endcap
            ENDCAP_BILINEAR_BASIS,       ///< use bilinear quads (4 cp) as end-caps
            ENDCAP_BSPLINE_BASIS,        ///< use BSpline basis patches (16 cp) as end-caps
            ENDCAP_GREGORY_BASIS,        ///< use Gregory basis patches (20 cp) as end-caps
        };

        Options(unsigned int maxIsolation=10) :
             maxIsolationLevel(maxIsolation),
             endCapType(ENDCAP_BSPLINE_BASIS) { }

        /// \brief Get endcap patch type
        EndCapType GetEndCapType() const { return (EndCapType)endCapType; }

        /// \brief Set endcap patch type
        void SetEndCapType(EndCapType e) { endCapType = e; }

        unsigned int maxIsolationLevel : 4, ///< Cap adaptive feature isolation to the given level (max. 10)
                     endCapType        : 3; ///< EndCapType
    };

    /// \brief Factory constructor for NodeTreeBuilder
    ///
    /// @param refiner              TopologyRefiner from which to generate patches
    ///
    /// @param patchTags            FacePatchTags corresponding to the (adaptive) refiner
    ///
    /// @param options              Options controlling the operation of the builder
    ///
    NodeTreeBuilder(TopologyRefiner const & refiner,
        PatchFaceTagVector const & patchTags,
           Options options=Options());

    ~NodeTreeBuilder();

    /// \brief Creates a node hierarchy for the topology of the given coarse face
    ///
    /// @param faceIndex            Index of the coarse face
    ///
    Characteristic * CreateCharacteristic(int faceIndex);

    /// \bried Debug printout
    void PrintTree() const;

private:

    //
    // Construct tree
    //

    void initializaLevelPatchTags();

    int extractNode(int levelIndex, int faceIndex);

    int extractRecursiveNode(int levelIndex, int faceIndex);

    void extractNgonNode(int faceIndex);

    Node * extractRegularNode(PatchFaceTag const & patchTag, int level, int face);

    Node * extractLimitNode(int level, int face);

    void printTree(Index faceIndex) const;

private:

    //
    // Flatten tree into chartacteristic table
    //

    bool nodeIsTerminal(int nodeIndex) const;

    void writeNode(int nodeIndex, int * offset, int * dataSize, void * data) const;

    void writeRegularNode(int nodeIndex, int * offset, int * dataSize, void * data) const;

    void writeLimitNode(int nodeIndex, int * offset, int * dataSize, void * data) const;

    void writeRecursiveNode(int nodeIndex, int * offset, int * dataSize, void * data) const;

    void writeTerminalNode(int nodeIndex, int * offset, int * dataSize, void * data) const;

    void writeCharacteristicTree(Characteristic * ch) const;

private:

    Options _options;

    TopologyRefiner const & _refiner;

    std::vector<PatchFaceTag const *> _levelPatchTags;
    std::vector<Index> _levelVertOffsets;

    union {
        EndCapBSplineBasisPatchFactory * _endCapBSpline;
        EndCapGregoryBasisPatchFactory * _endCapGregoryBasis;
    };

    StencilTable * _endcapStencils;
    StencilTable * _endcapVaryingStencils;

    std::vector<float> _sharpnessValues;

private:

    class NodeAllocator {

    public:

        NodeAllocator() {
            allocateBlock();
        }

        ~NodeAllocator() {
            for (int i=0; i<(int)_blocks.size(); ++i) {
                delete [] _blocks[i];
            }
        }

        void Reset() {
            _currentBlock = _currentNode = 0;
        }

        Node * Allocate() {
            _currentNode++;
            if (_currentNode==_blockSize) {
                allocateBlock();
            }
            return &_blocks[_currentBlock][_currentNode];
        }

    private:

        void allocateBlock() {
            _blocks.push_back(new Node[_blockSize]);
            _currentBlock = (int)_blocks.size()-1;
            _currentNode = 0;
        }

        int const _blockSize = 1000;

        int _currentBlock,
            _currentNode;

        std::vector<Node *> _blocks;
    };

    NodeAllocator _nodeAllocator;

    std::vector<Node *> _nodes;
};

NodeTreeBuilder::NodeTreeBuilder(
    TopologyRefiner const & refiner,
        PatchFaceTagVector const & patchTags,
            Options options) :
                _options(options), _refiner(refiner) {

    // create stencil tables for end caps w/ matching factories
    _endcapStencils = new StencilTable(0);
    _endcapVaryingStencils = new StencilTable(0);

    switch (_options.GetEndCapType()) {
        case Options::ENDCAP_GREGORY_BASIS:
            _endCapGregoryBasis =
                new EndCapGregoryBasisPatchFactory(
                    refiner, _endcapStencils, _endcapVaryingStencils);
            break;
        case Options::ENDCAP_BSPLINE_BASIS:
            _endCapBSpline =
                new EndCapBSplineBasisPatchFactory(
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

NodeTreeBuilder::~NodeTreeBuilder() {

    switch (_options.GetEndCapType()) {
        case Options::ENDCAP_GREGORY_BASIS:
            delete _endCapGregoryBasis;
            break;
        case Options::ENDCAP_BSPLINE_BASIS:
            delete _endCapBSpline;
            break;
        default:
            assert(0);
    }
}

inline bool isSharpnessEqual(float s1, float s2) { return (s1 == s2); }

//
//  Trivial anonymous helper functions:
//

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

// generate a regular limit patch node (includes boundaries, corners & single-crease patches)
Node *
NodeTreeBuilder::extractRegularNode(
    PatchFaceTag const & patchTag, int levelIndex, int faceIndex) {

    Node * node = _nodeAllocator.Allocate();

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
        level.gatherQuadRegularInteriorPatchPoints(faceIndex, patchVerts, 0 /* no rotation*/);
        if (patchTag.isSingleCrease) {
            boundaryMask = (1<<bIndex);
            sharpness = level.getEdgeSharpness((level.getFaceEdges(faceIndex)[bIndex]));
            sharpness = std::min(sharpness, (float)(_options.maxIsolationLevel-levelIndex));
            node->type = NODE_SEMI_SHARP;
        } else {
            node->type = NODE_REGULAR;
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
        node->type = NODE_BOUNDARY;
    } else if (patchTag.boundaryCount == 2) {
        // Expand corner patch vertices and rotate to restore correct orientation.
        static int const permuteCorner[4][16] = {
            { -1, -1, -1, -1, -1, 0, 1, 4, -1, 3, 2, 5, -1, 8, 7, 6 },
            { -1, -1, -1, -1, 8, 3, 0, -1, 7, 2, 1, -1, 6, 5, 4, -1 },
            { 6, 7, 8, -1, 5, 2, 3, -1, 4, 1, 0, -1, -1, -1, -1, -1 },
            { -1, 4, 5, 6, -1, 1, 2, 7, -1, 0, 3, 8, -1, -1, -1, -1 } };
        permutation = permuteCorner[bIndex];
        level.gatherQuadRegularCornerPatchPoints(faceIndex, patchVerts, bIndex);
        node->type = NODE_CORNER;
    } else {
        assert(patchTag.boundaryCount <= 2);
    }

    offsetAndPermuteIndices(patchVerts, 16, levelVertOffset, permutation, node->controlVerts);

    node->sharpnessIndex = assignSharpnessIndex(sharpness, _sharpnessValues);

    assert(node);
    return node;
}

// we reached the isolation limit : generate an end-cap patch node
Node *
NodeTreeBuilder::extractLimitNode(int levelIndex, int faceIndex) {

    assert(levelIndex==_refiner.GetMaxLevel());

    Node * node = _nodeAllocator.Allocate();
    node->type = NODE_LIMIT;

    Vtr::internal::Level const & level = _refiner.getLevel(levelIndex);

    PatchFaceTag const * levelPatchTags = _levelPatchTags[levelIndex];

    Index levelVertOffset = _levelVertOffsets[levelIndex];

    ConstIndexArray cvs;
    switch(_options.GetEndCapType()) {
        case Options::ENDCAP_GREGORY_BASIS: break;
            cvs = _endCapGregoryBasis->GetPatchPoints(&level, faceIndex, levelPatchTags, levelVertOffset);
            break;
        case Options::ENDCAP_BSPLINE_BASIS:
            cvs = _endCapBSpline->GetPatchPoints(&level, faceIndex, levelPatchTags, levelVertOffset);
            break;
        case Options::ENDCAP_BILINEAR_BASIS:
        default:
            assert(0);
    }

    // copy & offset the vertex stencil IDs as the end-cap stencil tables
    // will be concatenated
    assert(cvs.size()<=16);
    memcpy(node->controlVerts, cvs.begin(), cvs.size() * sizeof(Index));

    node->sharpnessIndex = assignSharpnessIndex(0.0f, _sharpnessValues);

    assert(node);
    return node;
}

// generate a recursive node : there is no limit patch, so we generate a
// NODE_RECURSIVE describing sub-patches
int
NodeTreeBuilder::extractRecursiveNode(int levelIndex, int faceIndex) {

    int nodeIndex = (int)_nodes.size();

    if (levelIndex && (levelIndex==_options.maxIsolationLevel)) {
        // reached max isolation : create a limit cap
        _nodes.push_back(extractLimitNode(levelIndex, faceIndex));
    } else {

        TopologyLevel const & level = _refiner.GetLevel(levelIndex);

        ConstIndexArray childFaces = level.GetFaceChildFaces(faceIndex);
        assert(childFaces.size()==4);

        Node * node = _nodeAllocator.Allocate();
        node->type = NODE_RECURSIVE;
        node->childFaceCount = 4;

        _nodes.push_back(node);

        for (int child=0; child<childFaces.size(); ++child) {
            node->children[child] = extractNode(levelIndex+1, childFaces[child]);
        }
    }
    return nodeIndex;
}

// generate an n-gon node : can only be applied to non-quad coarse faces
void
NodeTreeBuilder::extractNgonNode(int faceIndex) {

    TopologyLevel const & level = _refiner.GetLevel(0);

    ConstIndexArray childFaces = level.GetFaceChildFaces(faceIndex);
    assert(childFaces.size()!=4);

    Node * node = _nodeAllocator.Allocate();
    node->type = NODE_NGON;
    node->childFaceCount = childFaces.size();
    _nodes.push_back(node);

    for (int child=0; child<childFaces.size(); ++child) {
        extractNode(1, faceIndex);
    }
}

// recursively traverse an adaptively refined topology from
// level / face down, and accumulate tree nodes
int
NodeTreeBuilder::extractNode(int levelIndex, int faceIndex) {

    PatchFaceTag const & patchTag = _levelPatchTags[levelIndex][faceIndex];

    int nodeIndex = (int)_nodes.size();

    if (patchTag.hasPatch) {
        if (patchTag.isRegular) {
            _nodes.push_back(extractRegularNode(patchTag, levelIndex, faceIndex));
        } else {
            // reached max isolation : create a limit cap
            _nodes.push_back(extractLimitNode(levelIndex, faceIndex));
        }
    } else {
        if (levelIndex==0) {
            ConstIndexArray fverts = _refiner.GetLevel(levelIndex).GetFaceVertices(faceIndex);
            if (fverts.size()!=4) {
                extractNgonNode(faceIndex);
                return 0;
            }
        }
        extractRecursiveNode(levelIndex, faceIndex);
    }

    return nodeIndex;
}

void
NodeTreeBuilder::printTree(Index faceIndex) const {

    if (faceIndex>0) {
        printf(",\n");
    }
    printf("  \"face%d\" : [\n", faceIndex);
    for (int i=0; i<(int)_nodes.size(); ++i) {
        if (i>0) {
            printf(",\n");
        }
        printf("    \"node%d\" : ", i);
        _nodes[i]->Print();
    }
    printf("\n  ]");
    fflush(stdout);
}


//
// Write characteristic tree
//

bool
NodeTreeBuilder::nodeIsTerminal(int nodeIndex) const {

// XXXX
return false;

    Node const * node = _nodes[nodeIndex];
    if (node->type!=NODE_RECURSIVE) {
        return false;
    }
    int nrecursive = 0;
    for (int i=0; i<4; ++i) {
        Node const * child = _nodes[node->children[i]];
        switch (child->type) {
            case NODE_REGULAR: break;
            case NODE_RECURSIVE: ++nrecursive; break;
            default:
                return false;
        };        
        if (nrecursive>1) {
            return false;
        }
    }
    return true;
}

void
NodeTreeBuilder::writeTerminalNode(
    int nodeIndex, int * offset, int * dataSize, void * data) const {

    int dataOffset = *dataSize,
        * indices = (int *)((char *)data + *dataSize);

    for (;;) {

        Node const * node = _nodes[nodeIndex];

/*
        switch (node->type) {

            case NODE_LIMIT:
                if (data) {
                    // XXX copy CVs for end-cap patch
                }
                break;

            case NODE_REGULAR:

            case NODE_RECURSIVE:
                // XXX 25 CVs terminal node case (single EV)
                // can we do it with rotations ????
        };
*/
        break;
    }
}


//
// Memory layout:
// [[ support vert indices (16,12,9 ints) ]], [[ sharpness (1 float)]]
//
void
NodeTreeBuilder::writeRegularNode(
    int nodeIndex, int * offset, int * dataSize, void * data) const {

    Node const * node = _nodes[nodeIndex];

    bool isSemiSharp = false;
    int nsupports = 16;
    switch(node->type) {
        case NODE_REGULAR:
        case NODE_SEMI_SHARP: isSemiSharp = true; break;
        case NODE_BOUNDARY: nsupports = 12; break;
        case NODE_CORNER: nsupports = 9; break;
        default:
            assert(0);
    };

    if (data) {
        // XXXX
    }

    *dataSize += nsupports * sizeof(int);

    if (isSemiSharp) {
        *dataSize += sizeof(float);
        if (data) {
            // XXXX
        }
    }
}

void
NodeTreeBuilder::writeLimitNode(
    int nodeIndex, int * offset, int * dataSize, void * data) const {

}

void
NodeTreeBuilder::writeRecursiveNode(
    int nodeIndex, int * offset, int * dataSize, void * data) const {

    Node const * node = _nodes[nodeIndex];

    assert(node->type==NODE_RECURSIVE);

    if (nodeIsTerminal(nodeIndex)) {
        writeTerminalNode(nodeIndex, offset, dataSize, data);
    } else {
        for (int i=0; i<4; ++i) {
            if (data) {
            }
            int * childOffsets = (int *)((char *)data + *dataSize);
            *dataSize += 4 * sizeof(int);
            writeNode(node->children[i], &childOffsets[i], dataSize, data);
        }
    }
}

void
NodeTreeBuilder::writeNode(
    int nodeIndex, int * offset, int * dataSize, void * data) const {

    Node const * node = _nodes[nodeIndex];

    switch(node->type) {
        case NODE_REGULAR:
        case NODE_SEMI_SHARP:
        case NODE_BOUNDARY:
        case NODE_CORNER:
            writeRegularNode(nodeIndex, offset, dataSize, data);
            break;
        case NODE_LIMIT:
            writeLimitNode(nodeIndex, offset, dataSize, data);
            break;
        case NODE_RECURSIVE:
            writeRecursiveNode(nodeIndex, offset, dataSize, data);
            break;
        default:
            assert(0);
    };
}

void
NodeTreeBuilder::writeCharacteristicTree(Characteristic * ch) const {

    writeNode(0, nullptr, &ch->treeSize, nullptr);

    ch->tree = new int[ch->treeSize];

    int offset = 0;
    //writeNode(0, &ch->treeRoot, &offset, ch->tree);
}

Characteristic *
NodeTreeBuilder::CreateCharacteristic(int face) {

    _nodes.clear();

    _nodeAllocator.Reset();

    // recursively build nodes tree
    extractNode(0, face);

    printTree(face);

    Characteristic * ch = new Characteristic;

    writeCharacteristicTree(ch);

    return ch;
}


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

using namespace OpenSubdiv;


static void
testMe(ShapeDesc const & shapeDesc, int maxlevel=3) {

    Shape const * shape = Shape::parseObj(
        shapeDesc.data.c_str(), shapeDesc.scheme);

    // create Far mesh (topology)
    Sdc::SchemeType sdctype = GetSdcType(*shape);
    Sdc::Options    sdcoptions = GetSdcOptions(*shape);

    Far::TopologyRefiner * refiner =
        Far::TopologyRefinerFactory<Shape>::Create(*shape,
            Far::TopologyRefinerFactory<Shape>::Options(sdctype, sdcoptions));

    delete shape;

    // refine adaptively
    {
        Far::TopologyRefiner::AdaptiveOptions options(maxlevel);
        refiner->RefineAdaptive(options);
    }

    // identify patch types
    Far::PatchFaceTagVector patchTags;
    Far::PatchFaceTag::IdentifyAdaptivePatches(
        *refiner, patchTags, maxlevel, false /*single crease*/);

    // build trees
    {
        Far::NodeTreeBuilder::Options options(maxlevel);
        Far::NodeTreeBuilder builder(*refiner, patchTags, options);

        // build trees
        int nfaces = refiner->GetLevel(0).GetNumFaces();

        for (int face=0; face<nfaces; ++face) {

            Far::Characteristic * ch = builder.CreateCharacteristic(face);

            delete ch;
        }
    }
}



int main(int argc, char **argv) {

    std::string str;

    if (argc > 1) {
        std::ifstream ifs(argv[1]);
        if (ifs) {
            std::stringstream ss;
            ss << ifs.rdbuf();
            ifs.close();
            str = ss.str();
            g_defaultShapes.push_back(ShapeDesc(argv[1], str.c_str(), kCatmark));
        }
    }

    initShapes();

    ShapeDesc const & sdesc = g_defaultShapes[8]; // g_defaultShapes[42];
    int level=4;
printf("Shape='%s' (lvl=%d)\n", sdesc.name.c_str(), level);
    testMe(sdesc, level);
}
