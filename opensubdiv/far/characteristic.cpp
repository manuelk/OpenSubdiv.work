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

#include "../far/characteristic.h"
#include "../far/characteristicBuilder.h"
#include "../far/characteristicMap.h"
#include "../far/neighborhood.h"
#include "../far/patchBasis.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {


//
// Characteristic Node
//

//
// The hierarchical layout of adaptive sub-patches is encoded in "characteristic
// tree".  The nodes of each tree are serialized in a vector of integers. There
// are several types of nodes, each with its own encoding.
//
// Node types :
//
//  * NODE_REGULAR :
//    Leaf node describing a bspline patch
//
//  * NODE_END :
//    Leaf node describing an "end-cap" patch.
//
//  * NODE_RECURSIVE :
//    Branch node.
//
//  * NODE_TERMINAL :
//    Optimization node compressing several nodes for the case of irregular
//    faces that contain only a single extraordinary vertex.
//
// Node Layouts :
//
//   Field      | size | Content
//  ------------|:----:|----------------------------------------------------
//  REGULAR     |      |
//   descriptor | 1    | see NodeDescriptor
//   sharpness  | 1    | crease sharpness (single crease nodes only)
//   supports   | 16   | support indices
//  ------------|:----:|----------------------------------------------------
//  EMD         |      |
//   descriptor | 1    | see NodeDescriptor
//   supports   | 0-20 | support indices (depending on end-cap type)
//  ------------|:----:|----------------------------------------------------
//  Recursive   |      |
//   descriptor | 1    | see NodeDescriptor
//   offsets    | 4    | offsets to 4 children nodes
//  ------------|:----:|----------------------------------------------------
//  TERMINAL    |      |
//   descriptor | 1    | see NodeDescriptor
//   offsets    | 1    | offset to end-cap or terminal child node
//   supports   | 25   | support indices
//  ------------|:----:|----------------------------------------------------
//
// Sizes are in 'integers' (4 bytes)
//

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
    NodeDescriptor desc = GetDescriptor();
    switch (desc.GetType()) {
        case Characteristic::NODE_TERMINAL: return 1;
        case Characteristic::NODE_RECURSIVE: return 4;
        default:
            return 0;
    }
}

Characteristic::Node
Characteristic::Node::GetChildNode(int childIndex) const {

    NodeDescriptor desc = GetDescriptor();

    switch (desc.GetType()) {
        case Characteristic::NODE_TERMINAL:
            assert(childIndex==0);
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

float
Characteristic::Node::GetSharpness() const {

    assert(GetDescriptor().SingleCrease());

    float const * ptr = (float const *)(
        getNodeData() + sizeof(NodeDescriptor)/sizeof(int));
    return *ptr;
}

ConstIndexArray
Characteristic::Node::GetSupportIndices() const {

    int const * supportsPtr = getNodeData() + sizeof(NodeDescriptor)/sizeof(int);

    NodeDescriptor desc = GetDescriptor();
    switch (desc.GetType()) {
        case NODE_REGULAR : {
            if (desc.SingleCrease()) {
                supportsPtr += sizeof(float)/sizeof(int);
            }
            return ConstIndexArray(supportsPtr, 16);
        }
        case NODE_END: {
            EndCapType endType =
                GetCharacteristic()->GetCharacteristicMap()->GetEndCapType();
            int nsupports = 0;
            switch (endType) {
                case ENDCAP_BILINEAR_BASIS : nsupports = 0; break;
                case ENDCAP_BSPLINE_BASIS  : nsupports = 16; break;
                case ENDCAP_GREGORY_BASIS  : nsupports = 20; break;
                default:
                    assert(0);
            }
            return ConstIndexArray(supportsPtr, nsupports);
        }
        case NODE_TERMINAL: {
            ++supportsPtr; // skip child node offset
            return ConstIndexArray(supportsPtr, 25);
        } break;
        case NODE_RECURSIVE:
        default:
            return ConstIndexArray(nullptr, 0);
    }
}

void
Characteristic::Node::GetSupportIndices(
    unsigned char quadrant, Index * indices) const {

    assert(GetDescriptor().GetType()==NODE_TERMINAL);

    static int permuteTerminal[4][16] = {
        {0, 1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18},
        {1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19},
        {5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18, 20, 21, 22, 23}, // note : winding order is 0, 1, 3, 2
        {6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24},
    };

    // get all 25 indices
    ConstIndexArray supportIndices = GetSupportIndices();

    // copy the 16 support indices for quadrant
    for (int k=0; k<16; ++k) {
        indices[k] = supportIndices[permuteTerminal[quadrant][k]];
    }
}


int
Characteristic::Node::getNodeSize() const {

    int size = sizeof(NodeDescriptor)/sizeof(int);

    NodeDescriptor desc = GetDescriptor();
    switch (desc.GetType()) {

        case Characteristic::NODE_RECURSIVE:
            size += 4; // children offsets
            break;

        case Characteristic::NODE_REGULAR : {
            if (desc.SingleCrease()) {
                size += sizeof(float)/sizeof(int);
            }
            size += 16; // support indices
        } break;

        case Characteristic::NODE_END: {
            EndCapType endType =
                GetCharacteristic()->GetCharacteristicMap()->GetEndCapType();
            switch (endType) {
                case ENDCAP_NONE           : break;
                case ENDCAP_BILINEAR_BASIS : size += 4; break;
                case ENDCAP_BSPLINE_BASIS  : size += 16; break;
                case ENDCAP_GREGORY_BASIS  : size += 20; break;
                default:
                    assert(0);
            }
        } break;

        case Characteristic::NODE_TERMINAL: {
            size += 1;  // child offset
            size += 25; // support indices
        } break;

        default:
            assert(0);
            break;
    }
    return size;
}

//
// Characteristic
//

Characteristic::~Characteristic() {
    delete _tree;
    for (int i=0; i<(int)_neighborhoods.size(); ++i) {
        delete _neighborhoods[i];
    }
}

void
Characteristic::writeCharacteristicTree(CharacteristicBuilder & builder,
    int levelIndex, int faceIndex) {

    _treeSize = builder.GetTreeSize(levelIndex, faceIndex);
    _tree = new int[_treeSize];
#if 0
    // debug : paint memory
    for (int i=0; i<treeSize; ++i) {
        _tree[i]=-1;
    }
#endif
    builder.WriteTree(levelIndex, faceIndex, _tree);
    
    _treeOffset = builder.GetTreeOffset();
}



Characteristic::Node
Characteristic::GetTreeNode(float s, float t, unsigned char * quadrant) const {

    assert(_tree && sizeof(NodeDescriptor)==sizeof(int));

    // traverse the sub-patch tree to the (s,t) coordinates
    int offset = 0, corner = 0;
    NodeDescriptor desc = _tree[offset];

    Characteristic::NodeType ntype = desc.GetType();
    while (ntype==NODE_RECURSIVE || ntype==NODE_TERMINAL) {

        if (s>0.5f) { corner ^= 1; s = 1.0f - s; }
        if (t>0.5f) { corner ^= 2; t = 1.0f - t; }
        s *= 2.0f;
        t *= 2.0f;

        if (ntype==NODE_RECURSIVE) {
            // fetch next node
            offset = _tree[offset + 1 + corner];
            desc = _tree[offset];
        } else if (ntype==NODE_TERMINAL) {
            if (corner==desc.GetEvIndex()) {
                // traverse to end-cap patch
                offset = _tree[offset + 1];
                desc = _tree[offset];
            } else {
                // regular sub-patch : exit
                if (quadrant) {
                    *quadrant = corner;
                }
                break;
            }
        }
        ntype = desc.GetType();
    }
    return Node(this, offset);
}

Characteristic::Node
Characteristic::EvaluateBasis(float s, float t,
    float wP[], float wDs[], float wDt[], unsigned char * subpatch) const {

    unsigned char quadrant = 0;
    Node n = GetTreeNode(s, t, &quadrant);

    NodeDescriptor desc = n.GetDescriptor();

    int depth = desc.GetDepth() - (desc.NonQuadRoot() ? 1 : 0);

    PatchParam param;
    switch (desc.GetType()) {

       case NODE_REGULAR : {
            param.Set(/*face id*/ 0, desc.GetU(), desc.GetV(), depth,
                desc.NonQuadRoot(), desc.GetBoundaryMask(), 0);

            if (desc.SingleCrease()) {
                float sharpness = n.GetSharpness();
                internal::GetBSplineWeights(param, sharpness, s, t, wP, wDs, wDt);
            } else {
                internal::GetBSplineWeights(param, s, t, wP, wDs, wDt);
            }
        } break;

       case NODE_END : {
            param.Set(/*face id*/ 0, desc.GetU(), desc.GetV(), depth,
                desc.NonQuadRoot(), desc.GetBoundaryMask(), 0);

            switch (GetCharacteristicMap()->GetEndCapType()) {
                case ENDCAP_NONE :
                    break;
                case ENDCAP_BILINEAR_BASIS :
                    // XXXX manuelk TODO
                    assert(0);
                    break;
                case ENDCAP_BSPLINE_BASIS :
                    internal::GetBSplineWeights(param, s, t, wP, wDs, wDt);
                    break;
                case ENDCAP_GREGORY_BASIS :
                    internal::GetGregoryWeights(param, s, t, wP, wDs, wDt);
                    break;
                default:
                    assert(0);
            }
        } break;

       case NODE_TERMINAL : {
            unsigned short u = desc.GetU(),
                           v = desc.GetV();
            switch (quadrant) {
                case 0 :                 break;
                case 1 : { u+=1;       } break;
                case 2 : {       v+=1; } break; // ^ bitwise winding order !!!
                case 3 : { u+=1; v+=1; } break;
            }

            param.Set(/*face id*/ 0, u, v, depth+1, desc.NonQuadRoot(), 0, 0);

            internal::GetBSplineWeights(param, s, t, wP, wDs, wDt);

            if (subpatch) {
                *subpatch = quadrant;
            }
        } break;

        default:
            assert(0);
    }
    return n;
}

//
// Debug functions
//

static void
printNodeIndices(FILE * fout, ConstIndexArray cvs) {

    int stride = cvs.size()==16 ? 4 : 5;

    for (int i=0; i<cvs.size(); ++i) {
        if (i>0 && ((i%stride)!=0))
            fprintf(fout, " ");
        if ((i%stride)==0)
            fprintf(fout, "\\n");
        fprintf(fout, "%*d", 4, cvs[i]);
    }
}

inline size_t
hashNodeID(int charIndex, Characteristic::Node node) {
    size_t hash = node.GetTreeOffset() + ((size_t)charIndex << 32);
    return hash;
}

static void
printCharacteristicTreeNode(FILE * fout, Characteristic::Node node, int charIndex, bool showIndices=false) {

    typedef Characteristic::NodeDescriptor Descriptor;

    Descriptor const & desc = node.GetDescriptor();

    size_t nodeID = hashNodeID(charIndex, node);

    switch (desc.GetType()) {
        case Characteristic::NODE_REGULAR : {
                fprintf(fout, "  %zu [label=\"R\\n", nodeID);
                if (showIndices) {
                    printNodeIndices(fout, node.GetSupportIndices());
                }
                if (desc.SingleCrease()) {
                    fprintf(fout, "\\n\\nsharp=%f", node.GetSharpness());
                }
                fprintf(fout, "\" ");
                if (desc.SingleCrease()) {
                    fprintf(fout, " , style=filled, color=darksalmon");
                }
                fprintf(fout, ", shape=box]\n");
            } break;

        case Characteristic::NODE_END : {
                fprintf(fout, "  %zu [label=\"E\\n", nodeID);
                if (showIndices) {
                    printNodeIndices(fout, node.GetSupportIndices());
                }
                fprintf(fout, "\", shape=box, style=filled, color=darkorange]\n");
            } break;

        case Characteristic::NODE_RECURSIVE : {
                fprintf(fout, "  %zu [label=\"I\", shape=square, style=filled, color=dodgerblue]\n",nodeID );
                for (int i=0; i<4; ++i) {
                    Characteristic::Node child = node.GetChildNode(i);
                    printCharacteristicTreeNode(fout, child, charIndex, showIndices);
                    fprintf(fout, "  %zu -> %zu [label=\"%d\"]\n", nodeID, hashNodeID(charIndex, child), i);
                }
            } break;

        case Characteristic::NODE_TERMINAL : {
            fprintf(fout, "  %zu [shape=box, style=filled, color=grey, label=\"T", nodeID);
            if (showIndices) {
                printNodeIndices(fout, node.GetSupportIndices());
            }
            fprintf(fout, "\"]\n");

            Characteristic::Node child = node.GetChildNode();

            printCharacteristicTreeNode(fout, child, charIndex, showIndices);

            fprintf(fout, "  %zu -> %zu\n", nodeID, hashNodeID(charIndex, child));
        } break;
        default:
            assert(0);
    }
}

void
Characteristic::WriteTreeDiagraph(FILE * fout,
    int charIndex, bool showIndices, bool isSubgraph) const {
    if (isSubgraph) {
        fprintf(fout, "subgraph {\n");
    } else {
        fprintf(fout, "digraph {\n");
    }
    printCharacteristicTreeNode(fout, GetTreeRootNode(), charIndex, showIndices);
    fprintf(fout, "}\n");
}

//
// Neighborhoods
//

void
Characteristic::reserveNeighborhoods(int count) {
    _neighborhoods.reserve(count);
    _startEdges.reserve(count);
}

void
Characteristic::addNeighborhood(Neighborhood const * n, int startEdge) {
    _neighborhoods.push_back(n);
    _startEdges.push_back(startEdge);
}

void
Characteristic::shrink_to_fit() {
#if __cplusplus <= 199711L
    int count = GetNumNeighborhoods();
    _neighborhoods.resize(count);
    _startEdges.resize(count);
#else
    _neighborhoods.shrink_to_fit();
    _startEdges.shrink_to_fit();
#endif
}

int
Characteristic::FindEquivalentNeighborhood(Neighborhood const & n) const {
    for (int i=0; i<GetNumNeighborhoods(); ++i) {
        if (n.IsEquivalent(*_neighborhoods[i])) {
            return i;
        }
    }
    return INDEX_INVALID;
}



} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
