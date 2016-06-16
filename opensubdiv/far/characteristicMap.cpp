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

#include "../far/characteristicMap.h"

#include "../far/patchBasis.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {


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
            EndCapType endType =
                GetCharacteristic()->GetCharacteristicMap()->GetEndCapType();
            int nsupports = 0;
            switch (endType) {
                case ENDCAP_BSPLINE_BASIS : nsupports = 16; break;
                case ENDCAP_GREGORY_BASIS : nsupports = 20; break;
                default:
                    assert(0);
            }
            return ConstIndexArray(supportsPtr, nsupports);
        }
        case NODE_RECURSIVE:
        case NODE_TERMINAL:
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
            EndCapType endType =
                GetCharacteristic()->GetCharacteristicMap()->GetEndCapType();
            int nsupports = 0;
            switch (endType) {
                case ENDCAP_BSPLINE_BASIS : nsupports = 16; break;
                case ENDCAP_GREGORY_BASIS : nsupports = 20; break;
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

void
Characteristic::EvaluateBasis(Node n, float s, float t,
    float wP[], float wDs[], float wDt[]) const {

    NodeDescriptor desc = n.GetDescriptor();

    PatchParam param;
    param.Set(/*face id*/ 0, desc.GetU(), desc.GetV(), desc.GetDepth(), desc.NonQuadRoot(),
        desc.GetBoundaryMask(), desc.GetTransitionMask());

    if (desc.GetType()==NODE_REGULAR) {

        internal::GetBSplineWeights(param, s, t, wP, wDs, wDt);

    } else if (desc.GetType()==NODE_END) {

        EndCapType type =
            GetCharacteristicMap()->GetEndCapType();
        switch (type) {
            case ENDCAP_NONE :
                return;
            case ENDCAP_BSPLINE_BASIS :
                internal::GetBSplineWeights(param, s, t, wP, wDs, wDt);
                break;
            case ENDCAP_GREGORY_BASIS :
                internal::GetGregoryWeights(param, s, t, wP, wDs, wDt);
                break;
            default:
                assert(0);
        }
    } else {
        assert(0);
    }
}

Characteristic::Node
Characteristic::EvaluateBasis(float s, float t,
    float wP[], float wDs[], float wDt[]) const {

    Node n = GetTreeNode(s, t);

    EvaluateBasis(n, s, t, wP, wDs, wDt);
    
    return n;
}


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
