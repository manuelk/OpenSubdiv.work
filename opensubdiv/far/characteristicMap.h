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


#ifndef OPENSUBDIV3_FAR_CHARACTERISTIC_H
#define OPENSUBDIV3_FAR_CHARACTERISTIC_H

#include "../version.h"

#include "../far/types.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class CharacteristicMap;
class StencilTable;

///
///  \brief Stores a characteristic plan
///
/// * Topology trees :
///   A tree representation of the hierarchies of feature adaptive sub-patches.
///   The tree contains different types of nodes:
///
///   * Regular nodes :
///     Describes a regular sub-domain where the limit surface is a bicubic
///     B-spline.
///
///   * End nodes :
///     Nodes describing the surface around isolated features. While all "End-cap"
///     patches in a characteristic map must be of the same type, user can select
///     different types (bilinear, B-spline, gregory).
///     XXXX manuelk TODO bilinear
///
///   * Recursive nodes :
///     Connects to 4 child sub-domain nodes.
///
///   * Terminal nodes : 
///     A sub-patch tree optimization that allows the collapsing portions of a
///     topological tree into "terminal" nodes. Faces that contain a single
///     extraordinary vertex, but are otherwise "regular" (no boundaries,
///     no creases...) generate 3 regular sub-patches for each level of adaptive
///     isolation.
///     Each of these sub-patches requires 18 supports, however many of the
///     supports overlap, so that out of 54 supports, only 24 are not redundant.
///     Terminal nodes store those supports more efficiently.
///
///  For more details, see: 
///  "Efficient GPU Rendering of SUbdivision Surfaces using Adaptive Quadtrees"
///    W. Brainer, T. Foley, M. Mkraemer, H. Moreton, M. Niessner - Siggraph 2016
///
///  http://www.graphics.stanford.edu/~niessner/papers/2016/4subdiv/brainerd2016efficient.pdf
///
class Characteristic {

public:

    /// \brief Returns the map this characteristic belongs to
    CharacteristicMap const * GetCharacteristicMap() const { return _characteristicMap; }

public:

    //
    // Sub-patches Tree
    //

    //@{
    ///  @name Sub-patch tree access methods
    ///

    enum NodeType {
        NODE_REGULAR = 0,
        NODE_RECURSIVE = 1,
        NODE_TERMINAL = 2,
        NODE_END = 3,
    };

    /// NodeDescriptor
    ///
    /// Note : this descriptor *must* have the size of an int !
    ///
    /// Bitfield layout :
    ///
    ///  Field1       | Bits | Content
    ///  -------------|:----:|------------------------------------------------------
    ///  type         | 2    | type
    ///  nonquad      | 1    | whether the patch is the child of a non-quad face
    ///  singleCrease | 1    | true if "single crease" patch
    ///  depth        | 4    | the subdivision level of the node
    ///  transition   | 4    | transition edge mask encoding
    ///  boundary     | 4    | boundary edge mask encoding
    ///  v            | 8    | log2 value of u parameter at first patch corner
    ///  u            | 8    | log2 value of v parameter at first patch corner
    ///
    struct NodeDescriptor {

        /// \brief Translation constructor
        NodeDescriptor(int value=0) { field0=value; }

        void SetPatch(unsigned short type, unsigned short nonquad,
            unsigned short singleCrease, unsigned short depth,
                unsigned short boundary, unsigned short transition,
                    short u, short v) {
            field0 = ((v & 0xff)           << 24) |
                     ((u & 0xff)           << 16) |
                     ((boundary & 0xf)     << 12) |
                     ((transition & 0xf)   <<  8) |
                     ((depth & 0xf)        <<  4) |
                     ((singleCrease ? 1:0) <<  3) |
                     ((nonquad ? 1:0)      <<  2) |
                      (type & 0x3);
        }

        void SetRecursive(unsigned short depth) {
            field0 = ((depth & 0xf)        <<  4) |
                      (NODE_RECURSIVE & 0x3);
        }

        void SetTerminal(unsigned short nonquad,
            unsigned short depth, unsigned short evIndex, short u, short v) {
            field0 = ((v & 0xff)           << 24) |
                     ((u & 0xff)           << 16) |
                     ((evIndex & 0xf)      << 12) |
                     ((depth & 0xf)        <<  4) |
                     ((nonquad ? 1:0)      <<  2) |
                      (NODE_TERMINAL & 0x3);
        }

        NodeDescriptor & operator=(int value) { field0 = value; return *this; }

        /// \brief Resets everything to 0
        void Clear() { field0 = 0; }

        /// \brief Returns the type for the sub-patch.
        NodeType GetType() const { return (NodeType)(field0 & 0x3); }

        /// \brief Returns the level of subdivision of the sub-patch
        unsigned short GetDepth() const { return  (unsigned short)((field0 >> 4) & 0xf); }

        // XXXX manuelk characteristic plan evaluation does not need transition
        // masks -> we should recover those 4 bits and use them for (u,v) instead...

        /// \brief Returns the transition edge encoding for the patch.
        unsigned short GetTransitionMask() const { return (unsigned short)((field0 >> 8) & 0xf); }

        /// \brief Returns the number of transition edges in the sub-patch (-1 for invalid mask)
        unsigned short GetTransitionCount() const;

        /// \brief Returns the boundary edge encoding for the sub-patch.
        unsigned short GetBoundaryMask() const { return (unsigned short)((field0 >> 12) & 0xf); }

        /// \brief Returns the number of boundary edges in the sub-patch (-1 for invalid mask)
        unsigned short GetBoundaryCount() const;

        /// \brief Returns local index of the extraordinary vertex in a terminal patch
        unsigned short GetEvIndex() const { return (unsigned short)((field0 >> 12) & 0xf); }

        /// \brief True if the parent coarse face is a non-quad
        bool NonQuadRoot() const { return (field0 >> 2) & 0x1; }

        /// \brief Returns true if the patch is of "single crease" type
        bool SingleCrease() const { return (field0 >> 3) & 0x1; }

        /// \brief Returns the log2 value of the u parameter at the top left corner of
        /// the patch
        unsigned short GetU() const { return (unsigned short)((field0 >> 16) & 0xff); }

        /// \brief Returns the log2 value of the v parameter at the top left corner of
        /// the patch
        unsigned short GetV() const { return (unsigned short)((field0 >> 24) & 0xff); }

        /// \brief Returns the fraction of normalized parametric space covered by the
        /// sub-patch.
        float GetParamFraction() const;

        /// The (u,v) pair is normalized to this sub-parametric space.
        ///
        /// @param u  u parameter
        /// @param v  v parameter
        ///
        void Normalize( float & u, float & v ) const;

        int field0:32;
    };

    /// Tree Node
    ///
    /// note : The burden is on the client to check whether a particular accessor
    ///        method can be applied on a given node. If the node is of the wrong
    ///        type, behavior will be "undefined".
    class Node {

    public:

        Node() { }

        /// \brief Returns the node descriptor
        NodeDescriptor GetDescriptor() const {
            return _characteristic->_tree[_treeOffset];
        }

        /// \brief Returns the number of children nodes
        int GetNumChildrenNodes() const;

        /// \brief Returns the node's child at index
        /// note : 4 children for RECURSIVE nodes, 1 child for TERMINAL nodes
        Node GetChildNode(int childIndex=0) const;

        /// \brief Returns the creased edge sharpness
        /// note : the value is undefined for any node other than REGULAR
        ///        with a SingleCrease flag set to true
        float GetSharpness() const;

        /// \brief Returns a pointer to the indices of the support points
        ConstIndexArray GetSupportIndices() const;

        /// \brief Returns a pointer to the characteric that owns this node
        Characteristic const * GetCharacteristic() const { return _characteristic; }

        /// \brief Returns the node's offset
        int GetTreeOffset() const { return _treeOffset; }

        /// \brief Returns the next node in the tree (serial traversal)
        /// note : loops back to root node
        Node operator ++ ();

        /// \brief Returns true if the nodes are identical
        bool operator == (Node const & other) const;

    private:

        int const * getNodeData() const {
            return &_characteristic->_tree[_treeOffset];
        }
        
        int getNodeSize() const;

        friend class Characteristic;

        Node(Characteristic const * ch, int treeOffset) :
            _characteristic(ch), _treeOffset(treeOffset) { }

        Characteristic const * _characteristic;
        int _treeOffset;
    };


    /// \brief Returns the size (in bytes) of the patches tree
    int GetTreeSize() const { return _treeSize; }

    /// \brief Returns the tree data
    int const * GetTreeData() const { return _tree; }

    /// \brief Returns a pointer to the root node of the sub-patches tree
    Node GetTreeRootNode() const { return Node(this, 0); }

    /// \brief Returns a the node corresponding to the sub-patch at the given (s,t) location
    Node GetTreeNode(float s, float t, unsigned char * quadrant=0) const;

    /// \brief Returns the node at the given offset in the serialized tree
    Node GetTreeNode(int treeOffset) const { return Node(this, treeOffset); }

    /// \brief Returns the index of the node in the serialized tree
    int GetNodeIndex(Node node) const {
        Node it = GetTreeRootNode();
        for (int index=0; it.GetTreeOffset()<GetTreeSize(); ++index, ++it) {
            if (it==node) {
                return index;
            }
        }
        return -1;
    }
    //@}

public:

    //@{
    ///  @name Evaluation methods
    ///

    /// \brief Evaluate basis functions for position and first derivatives at a
    /// given (s,t) parametric location of a patch.
    ///
    /// @param s       Patch coordinate (in coarse face normalized space)
    ///
    /// @param t       Patch coordinate (in coarse face normalized space)
    ///
    /// @param wP      Weights (evaluated basis functions) for the position
    ///
    /// @param wDs     Weights (evaluated basis functions) for derivative wrt s
    ///
    /// @param wDt     Weights (evaluated basis functions) for derivative wrt t
    ///
    /// @return        The leaf node pointing to the sub-patch evaluated
    ///
    Node EvaluateBasis(float s, float t,
        float wP[], float wDs[], float wDt[], unsigned char * subpatch=0) const;

    //@}

private:

    // The sub-patch "tree" is stored as a linear buffer of integers for
    // efficient look-up & traversal on a GPU. Use the Node class to traverse
    // the tree and access each node's data.
    int * _tree,
          _treeSize;

private:

    friend class CharacteristicBuilder;
    friend class CharacteristicMapFactory;

    CharacteristicMap const * _characteristicMap;
};


///
///  \brief Stores topology characteristic plans
///
class CharacteristicMap {

public:

    //@{
    ///  @name Characteristics
    ///
    /// \anchor arrays_of_characteristics
    ///
    int GetNumCharacteristics() const {
        return (int)_characteristics.size();
    }

    Characteristic const & GetCharacteristic(Index charIndex) const {
        return _characteristics[charIndex];
    }

    //@}

    //@{
    ///  @name change of basis patches
    ///
    /// \anchor change_of_basis_patches
    ///
    /// \brief Accessors for change of basis patch points
    ///
    /// \brief Returns the stencil table to get change of basis patch points.
    StencilTable const * GetLocalPointStencilTable() const {
        return _localPointStencils;
    }

    /// \brief Returns the varying stencil table for the change of basis patch
    ///        points.
    StencilTable const * GetLocalPointVaryingStencilTable() const {
        return _localPointVaryingStencils;
    }
    //@}

    EndCapType GetEndCapType() const { return EndCapType(_endCapType); }

private:

    friend class CharacteristicMapFactory;

    CharacteristicMap(EndCapType endcaps) :
        _endCapType(endcaps), _localPointStencils(0), _localPointVaryingStencils(0) { }

private:

    // flags
    unsigned int _endCapType:2;

    // XXXX this eventually will be a map : right now it's just 1 characteristic per face
    std::vector<Characteristic> _characteristics;

    StencilTable const * _localPointStencils,        // endcap basis conversion stencils
                       * _localPointVaryingStencils; // endcap varying stencils (for convenience)
};

inline Characteristic::Node
Characteristic::Node::operator ++ () {
    _treeOffset += getNodeSize();
    if (_treeOffset > GetCharacteristic()->GetTreeSize()) {
        _treeOffset = 0;
    }
    return *this;
}

inline bool
Characteristic::Node::operator == (Characteristic::Node const & other) const {
    return _characteristic == other._characteristic &&
        _treeOffset == other._treeOffset;
}

inline float
Characteristic::NodeDescriptor::GetParamFraction( ) const {
    if (NonQuadRoot()) {
        return 1.0f / float( 1 << (GetDepth()-1) );
    } else {
        return 1.0f / float( 1 << GetDepth() );
    }
}

inline void
Characteristic::NodeDescriptor::Normalize( float & u, float & v ) const {

    float frac = GetParamFraction();

    // top left corner
    float pu = (float)GetU()*frac;
    float pv = (float)GetV()*frac;

    // normalize u,v coordinates
    u = (u - pu) / frac,
    v = (v - pv) / frac;
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_CHARACTERISTIC_FACTORY_H */
