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

    Node(int parent=Far::INDEX_INVALID) {
        memset(this, 0, sizeof(Node));
        parentNode = parent;
    }

    void Print() const;

    NodeType type;

    int sharpnessIndex;    // single crease sharpness

    int parentNode,
        childFaceCount;

    Index controlVerts[16];
};

void Node::Print() const {
    printf("{ \"type\":\"%s\"\t, \"parent\": %2d, \"nchildren\":%2d",
        NodeTypeNames[type], parentNode, childFaceCount);

    printf(", \"cvs\":[");
    for (int j=0; j<16; ++j) {
        if (j>0) printf(", ");
        printf("%d", controlVerts[j]);
    }
    printf("] }");
}

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
    /// @param tree                 Vector of tree nodes (root at index 0)
    ///
    void CreateTree(int faceIndex, std::vector<Node> & tree);

    /// \bried Debug printout
    void PrintTree() const;

private:

    void initializaLevelPatchTags();

    void extractNode(int levelIndex, int faceIndex, int parentNode);

    void extractNgonNode(int faceIndex);

    void extractRecursiveNode(int levelIndex, int faceIndex, int parentNode);

    Node * extractRegularNode(PatchFaceTag const & patchTag, int level, int face, int parentNode);

    Node * extractLimitNode(int level, int face, int parentNode);

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

        _levelPatchTags[i] = & patchTags[levelFaceOffset];
        _levelVertOffsets[i] = levelVertOffset;

        TopologyLevel const & level = _refiner.GetLevel(i);
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
    PatchFaceTag const & patchTag, int levelIndex, int faceIndex, int parentNode) {

    Node * node = new Node(parentNode);

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
NodeTreeBuilder::extractLimitNode(int levelIndex, int faceIndex, int parentNode) {

    assert(levelIndex==_refiner.GetMaxLevel());

    Node * node = new Node(parentNode);
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
void
NodeTreeBuilder::extractRecursiveNode(int levelIndex, int faceIndex, int parentNode) {

    if (levelIndex && (levelIndex==_options.maxIsolationLevel)) {

        // reached max isolation : create a limit cap
        _nodes.push_back(extractLimitNode(levelIndex, faceIndex, parentNode));
    } else {

        TopologyLevel const & level = _refiner.GetLevel(levelIndex);

        ConstIndexArray childFaces = level.GetFaceChildFaces(faceIndex);
        assert(childFaces.size()==4);

        Node * node = new Node(parentNode);
        node->type = NODE_RECURSIVE;
        node->childFaceCount = 4;
        _nodes.push_back(node);

        int nodeIndex = (int)_nodes.size()-1;

        for (int child=0; child<childFaces.size(); ++child) {
            extractNode(levelIndex+1, childFaces[child], nodeIndex);
        }
    }
}

// generate an n-gon node : can only be applied to non-quad coarse faces
void
NodeTreeBuilder::extractNgonNode(int faceIndex) {

    TopologyLevel const & level = _refiner.GetLevel(0);

    ConstIndexArray childFaces = level.GetFaceChildFaces(faceIndex);
    assert(childFaces.size()!=4);

    Node * node = new Node(Far::INDEX_INVALID);
    node->type = NODE_NGON;
    node->childFaceCount = childFaces.size();
    _nodes.push_back(node);

    for (int child=0; child<childFaces.size(); ++child) {
        extractNode(1, faceIndex, 0);
    }
}

// recursively traverse an adaptively refined topology from
// level / face down, and accumulate tree nodes
void
NodeTreeBuilder::extractNode(int levelIndex, int faceIndex, int parentNode) {

    PatchFaceTag const & patchTag = _levelPatchTags[levelIndex][faceIndex];

    if (patchTag.hasPatch) {
        if (patchTag.isRegular) {
            _nodes.push_back(
                extractRegularNode(patchTag, levelIndex, faceIndex, parentNode));
        } else {
            // reached max isolation : create a limit cap
            _nodes.push_back(
                extractLimitNode(levelIndex, faceIndex, parentNode));
        }
    } else {
        if (levelIndex==0) {
            ConstIndexArray fverts = _refiner.GetLevel(levelIndex).GetFaceVertices(faceIndex);
            if (fverts.size()!=4) {
                extractNgonNode(faceIndex);
                return;
            }
        }
        extractRecursiveNode(levelIndex, faceIndex, parentNode);
    }

}

void
NodeTreeBuilder::CreateTree(int face, std::vector<Node> & tree) {

    extractNode(0, face, Far::INDEX_INVALID);

    // pack nodes into contiguous memory
    // xxxx is this really necessary if we use this to build patches ???
    tree.resize(_nodes.size());
    for (int i=0; i<(int)_nodes.size(); ++i) {
        memcpy(&tree[i], _nodes[i], sizeof(Node));
        delete _nodes[i];
    }
    _nodes.clear();
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

        std::vector<std::vector<Far::Node> > trees(nfaces);

printf("[");
        for (int face=0; face<nfaces; ++face) {
            std::vector<Far::Node> & tree = trees[face];
            builder.CreateTree(face, tree);

if (face>0) printf(",\n");

printf("[\n");
for (int i=0; i<(int)tree.size(); ++i) {
if (i>0) printf(",\n");
printf("%d ",i); tree[i].Print();
}
printf("]\n");

        }
printf("]");

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
