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

#include "../far/subdivisionPlan.h"
#include "../far/subdivisionPlanBuilder.h"
#include "../far/topologyMap.h"
#include "../far/neighborhood.h"
#include "../far/patchBasis.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {


//
// Subdivision Plan Node
//

//
// The hierarchical layout of adaptive sub-patches is encoded in a subdivision
// plan tree.  The nodes of each tree are serialized in a vector of integers.
// There are several types of nodes, each with its own encoding.
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
//   Field         | size | Content
//  ---------------|:----:|----------------------------------------------------
//  REGULAR        |      |
//   descriptor    | 1    | see NodeDescriptor
//   first support | 1    | local index of the first support stencil
//   sharpness     | 1    | crease sharpness (single crease nodes only)
//  ---------------|:----:|----------------------------------------------------
//  EMD            |      |
//   descriptor    | 1    | see NodeDescriptor
//   first support | 1    | local index of the first support stencil
//  ---------------|:----:|----------------------------------------------------
//  Recursive      |      |
//   descriptor    | 1    | see NodeDescriptor
//   first support | 1    | local index of the first support stencil (dyn. isolation)
//   child nodes   | 4    | offsets to the 4 children nodes
//  ---------------|:----:|----------------------------------------------------
//  TERMINAL       |      |
//   descriptor    | 1    | see NodeDescriptor
//   first support | 1    | local index of the first support stencil
//   child node    | 1    | offset to the child node (END or TERMINAL)
//  ---------------|:----:|----------------------------------------------------
//
// Sizes are in 'integers' (4 bytes)
//

unsigned short
SubdivisionPlan::NodeDescriptor::GetBoundaryCount() const {
    // patches cannot have more than 2 boundaries : return -1 if more than 2 bits are set
    // 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
    // 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111
    static int masks[] = { 0,  1,  1,  2,  1,  2,  2, -1,
                           1,  2,  2, -1,  2, -1, -1, -1,  };
    return masks[GetBoundaryMask()];
}

int
SubdivisionPlan::Node::GetNumChildrenNodes() const {
    NodeDescriptor desc = GetDescriptor();
    switch (desc.GetType()) {
        case SubdivisionPlan::NODE_TERMINAL: return 1;
        case SubdivisionPlan::NODE_RECURSIVE: return 4;
        default:
            return 0;
    }
}

float
SubdivisionPlan::Node::GetSharpness() const {
    assert(GetDescriptor().SingleCrease());
    return *(float const *)(getNodeData() + 2);
}

Index
SubdivisionPlan::Node::getFirstSupportIndex() const {
    return *(getNodeData() + 1);
}

int
SubdivisionPlan::Node::GetNumSupports(int quadrant, short maxLevel) const {

    NodeDescriptor desc = GetDescriptor();

    NodeType type = desc.GetType();

    if (desc.GetDepth()>=maxLevel &&
        (type==NODE_RECURSIVE || type==NODE_TERMINAL)) {
        // if we hit the dynamic isolation level limit, then force the basis
        // to end-cap patch for terminal & recursive nodes
        type = NODE_END;
    }

    switch (type) {
        case NODE_REGULAR:
            return 16;
        case NODE_TERMINAL:
            // note : terminal nodes hold 25 supports, but the public-facing
            // API client code to select which regular patch to use and
            // automatically reduces the 25-set to 16 indices
            return desc.GetEvIndex()!=(short)quadrant ? 16 : 0;
        case NODE_END: {
            TopologyMap const & topomap =
                GetSubdivisionPlan()->GetTopologyMap();
            return getNumEndCapSupports(topomap.GetEndCapType());
        };
        default:
            return 0;
    }
}

Index
SubdivisionPlan::Node::GetSupportIndex(int supportIndex, int evIndex, short maxLevel) const {

    Index firstSupport = getFirstSupportIndex();

    NodeDescriptor desc = GetDescriptor();
    switch (desc.GetType()) {
        case NODE_REGULAR:
        case NODE_END:
            return firstSupport + supportIndex;
        case NODE_TERMINAL: {
            if (desc.GetDepth()>=maxLevel) {
                // if we hit the dynamic isolation cap, return the end-cap
                // support stored at the end of the regular supports
                firstSupport += 25;
            } else {
                // note : winding order is 0, 1, 3, 2 !
                static int permuteTerminal[4][16] = {
                    {0, 1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18},
                    {1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19},
                    {5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18, 20, 21, 22, 23},
                    {6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24},
                };
                assert(evIndex!=desc.GetEvIndex() && evIndex>=0 && evIndex<4);
                supportIndex = permuteTerminal[evIndex][supportIndex];
            }
            return firstSupport + supportIndex;
        }
        case NODE_RECURSIVE:
            return desc.GetDepth()>=maxLevel ?
                firstSupport + supportIndex : INDEX_INVALID;
    }
    assert(0);
    return INDEX_INVALID;
}

SubdivisionPlan::Support
SubdivisionPlan::Node::GetSupport(
    int supportIndex, int evIndex, short maxLevel) const {
    return GetSubdivisionPlan()->GetSupport(
        GetSupportIndex(supportIndex, evIndex, maxLevel));
}

//
// Subdivision Plan
//

SubdivisionPlan::~SubdivisionPlan() {
    // Plans own the lifespan of their neighborhoods : we need to
    // delete them
    for (int i=0; i<(int)_neighborhoods.size(); ++i) {
        free((void *)_neighborhoods[i]);
    }
}

SubdivisionPlan::Node
SubdivisionPlan::GetTreeNode(float s, float t, unsigned char * quadrant, short maxLevel) const {

    assert(sizeof(NodeDescriptor)==sizeof(int));

    // traverse the sub-patch tree to the (s,t) coordinates
    int offset = 0, corner = 0;
    NodeDescriptor desc = _tree[offset];

    SubdivisionPlan::NodeType ntype = desc.GetType();
    while (ntype==NODE_RECURSIVE || ntype==NODE_TERMINAL) {

        if (maxLevel>=0 && (short)desc.GetDepth()==maxLevel) {
            break;
        }

        if (s>0.5f) { corner ^= 1; s = 1.0f - s; }
        if (t>0.5f) { corner ^= 2; t = 1.0f - t; }
        s *= 2.0f;
        t *= 2.0f;

        if (ntype==NODE_RECURSIVE) {
            // fetch child node
            offset = _tree[offset + 2 + corner];
            desc = _tree[offset];
        } else if (ntype==NODE_TERMINAL) {
            if (corner==desc.GetEvIndex()) {
                // traverse to end-cap patch
                offset = _tree[offset + 2];
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

inline void
evaluateEndCapBasis(SubdivisionPlan::EndCapType type, PatchParam param,
    float s, float t, float wP[], float wDs[], float wDt[]) {

    switch (type) {
        case SubdivisionPlan::ENDCAP_NONE :
            break;
        case SubdivisionPlan::ENDCAP_BILINEAR_BASIS :
            internal::EvaluatePatchBasis<float>(PatchDescriptor::QUADS, param, s, t, wP, wDs, wDt);
            break;
        case SubdivisionPlan::ENDCAP_BSPLINE_BASIS :
            internal::EvaluatePatchBasis<float>(PatchDescriptor::REGULAR, param, s, t, wP, wDs, wDt);
            break;
        case SubdivisionPlan::ENDCAP_GREGORY_BASIS :
            internal::EvaluatePatchBasis<float>(PatchDescriptor::GREGORY, param, s, t, wP, wDs, wDt);
            break;
        default:
            assert(0);
    }
}

SubdivisionPlan::Node
SubdivisionPlan::EvaluateBasis(float s, float t,
    float wP[], float wDs[], float wDt[], unsigned char * subpatch, short maxLevel) const {

    unsigned char quadrant = 0;
    Node n = GetTreeNode(s, t, &quadrant, maxLevel);

    NodeDescriptor desc = n.GetDescriptor();

    NodeType type = desc.GetType();


    int depth = desc.GetDepth(); // - (desc.NonQuadRoot() ? 1 : 0);

    PatchParam param;

    if (hasDynamicIsolation() && (depth>=maxLevel) &&
        (type==NODE_RECURSIVE || type==NODE_TERMINAL)) {

        unsigned short u = desc.GetU(),
                       v = desc.GetV();
        if (type==NODE_TERMINAL) {
            u = u >> 1;
            v = v >> 1;
        }
        param.Set(INDEX_INVALID, u, v, depth, desc.NonQuadRoot(), 0, 0, true);

        evaluateEndCapBasis(getEndCapType(), param, s, t, wP, wDs, wDt);
    } else {

        switch (type) {

            case NODE_REGULAR : {
                param.Set(INDEX_INVALID, desc.GetU(), desc.GetV(), depth,
                    desc.NonQuadRoot(), desc.GetBoundaryMask(), 0, true);
                float sharpness = desc.SingleCrease() ? n.GetSharpness() : 0.f;
                internal::EvaluatePatchBasis<float>(
                    PatchDescriptor::REGULAR, param, s, t, wP, wDs, wDt, 0, 0, 0, sharpness);
            } break;

            case NODE_END : {
                param.Set(INDEX_INVALID,desc.GetU(), desc.GetV(), depth,
                    desc.NonQuadRoot(), desc.GetBoundaryMask(), 0, true);
                evaluateEndCapBasis(getEndCapType(), param, s, t, wP, wDs, wDt);
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
                param.Set(INDEX_INVALID, u, v, depth+1, desc.NonQuadRoot(), 0, 0, true);
                internal::EvaluatePatchBasis<float>(PatchDescriptor::REGULAR, param, s, t, wP, wDs, wDt);
            } break;

            default:
                assert(0);
        }
    }
    if (subpatch) {
        *subpatch = quadrant;
    }
    return n;
}


inline bool
SubdivisionPlan::hasDynamicIsolation() const {
    return (bool)_topologyMap.GetOptions().useDynamicIsolation;
}

inline SubdivisionPlan::EndCapType
SubdivisionPlan::getEndCapType() const {
    return GetTopologyMap().GetEndCapType();
}

//
// Debug functions
//

#if 0
static void
printNodeIndices(FILE * fout, SubdivisionPlan::Node node) {

    SubdivisionPlan const * plan = node.GetSubdivisionPlan();

    SubdivisionPlan::NodeDescriptor desc = node.GetDescriptor();

    int numSupports = node.getNumSupports(desc.GetType()),
        stride = (numSupports==16) ? 4 : 5;

    Index firstSupport = node.getFirstSupportIndex();
    for (int i=0; i<numSupports, ++i) {
        SubdivisionPlan::Support support = plan->GetSupport(firstSupport+1);
        if (i>0 && ((i%stride)!=0))
            fprintf(fout, " ");
        if ((i%stride)==0)
            fprintf(fout, "\\n");
    }
}
#endif

inline size_t
hashNodeID(int planIndex, SubdivisionPlan::Node node) {
    size_t hash = node.GetTreeOffset() + ((size_t)planIndex << 32);
    return hash;
}

static void
printTreeNode(FILE * fout, SubdivisionPlan::Node node, int planIndex) {

    typedef SubdivisionPlan::NodeDescriptor Descriptor;

    Descriptor const & desc = node.GetDescriptor();

    size_t nodeID = hashNodeID(planIndex, node);

    switch (desc.GetType()) {
        case SubdivisionPlan::NODE_REGULAR : {
            fprintf(fout, "  %zu [label=\"REG\\ntofs=%d\\nfsup=%d",
                nodeID, node.GetTreeOffset(), node.getFirstSupportIndex());
            if (desc.SingleCrease()) {
                fprintf(fout, "\\n\\nsharp=%f", node.GetSharpness());
            }
            fprintf(fout, "\", shape=box, style=filled, color=%s]\n", desc.SingleCrease() ? "darksalmon" : "white");
        } break;

        case SubdivisionPlan::NODE_END : {
            fprintf(fout, "  %zu [label=\"END\\ntofs=%d\\nfsup=%d",
                nodeID, node.GetTreeOffset(), node.getFirstSupportIndex());
            fprintf(fout, "\", shape=box, style=filled, color=darkorange]\n");
        } break;

        case SubdivisionPlan::NODE_RECURSIVE : {
            fprintf(fout, "  %zu [label=\"REC\\ntofs=%d\\nfsup=%d",
                nodeID, node.GetTreeOffset(), node.getFirstSupportIndex());
            fprintf(fout, "\", shape=square, style=filled, color=dodgerblue]\n");
            for (int i=0; i<4; ++i) {
                SubdivisionPlan::Node child = node.GetNodeChild(i);
                printTreeNode(fout, child, planIndex);
                fprintf(fout, "  %zu -> %zu [label=\"%d\"]\n", nodeID, hashNodeID(planIndex, child), i);
            }
        } break;

        case SubdivisionPlan::NODE_TERMINAL : {
            fprintf(fout, "  %zu [label=\"TRM\\ntofs=%d\\nfsup=%d",
                nodeID, node.GetTreeOffset(), node.getFirstSupportIndex());
            fprintf(fout, "\", shape=box, style=filled, color=grey]\n");
            SubdivisionPlan::Node child = node.GetNodeChild();
            printTreeNode(fout, child, planIndex);
            fprintf(fout, "  %zu -> %zu\n", nodeID, hashNodeID(planIndex, child));
        } break;
        default:
            assert(0);
    }
}

void
SubdivisionPlan::WriteTreeDigraph(FILE * fout,
    int planIndex, bool showIndices, bool isSubgraph) const {

    if (isSubgraph) {
        fprintf(fout, "subgraph cluster_%d {\n", planIndex);
        fprintf(fout, "  label = \"Plan %d\"; style=filled; color=lightgrey;\n", planIndex);
    } else {
        fprintf(fout, "digraph {\n");
    }
    printTreeNode(fout, GetTreeRootNode(), planIndex);
    fprintf(fout, "}\n");
}

//
// Neighborhoods
//

void
SubdivisionPlan::reserveNeighborhoods(int count) {
    _neighborhoods.reserve(count);
    _startEdges.reserve(count);
}

void
SubdivisionPlan::addNeighborhood(Neighborhood const * n, int startEdge) {
    _neighborhoods.push_back(n);
    _startEdges.push_back(startEdge);
}

void
SubdivisionPlan::shrink_to_fit() {
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
SubdivisionPlan::FindEquivalentNeighborhood(Neighborhood const & n) const {
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
