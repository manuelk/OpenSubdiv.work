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

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

namespace internal {
    class CharacteristicBuilder;
}

class CharacteristicMap;
class Neighborhood;

///
///  \brief Stores a subdivision "characteristic"
///
/// * Topology trees :
///   A tree representation of the hierarchies of feature adaptive sub-patches.
///   The nodes are stored in a flat array of integers, with offsets pointing
///   to children nodes.
///
///   The tree contains 4 different types of nodes:
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
///     See domain winding notes.
///
///   * Terminal nodes :
///     A sub-patch tree optimization that allows the collapsing portions of a
///     topological tree into "terminal" nodes. Faces that contain a single
///     extraordinary vertex, but are otherwise "regular" (no boundaries,
///     no creases...) generate 3 regular sub-patches for each level of adaptive
///     isolation.
///     Each of these sub-patches requires 16 supports, however many of the
///     supports overlap, so that out of 48 supports, only 24 are not redundant.
///     Terminal nodes store those supports more efficiently.
///
/// Notes:
///
///   * Domain winding
///     Sub-domain winding in topology trees does not follow the general
///     sequential winding order used elsewhere in OpenSubdiv. Instead, the
///     domains of sub-patches are stored with a "Z" winding order.
///     Winding patterns:
///       Sequential    ^ Bitwise
///       +---+---+     +---+---+
///       | 3 | 2 |     | 2 | 3 |
///       +---+---+     +---+---+
///       | 0 | 1 |     | 0 | 1 |
///       +---+---+     +---+---+
///     This winding pattern allows for faster traversal by using simple
///     '^' bitwise operators.
///
///  XXXX manuelk GPU-side gains may not be worth the complexity...
///
///  For more details, see:
///  "Efficient GPU Rendering of SUbdivision Surfaces using Adaptive Quadtrees"
///    W. Brainer, T. Foley, M. Mkraemer, H. Moreton, M. Niessner - Siggraph 2016
///
///  http://www.graphics.stanford.edu/~niessner/papers/2016/4subdiv/brainerd2016efficient.pdf
///
class Characteristic {

public:

    class Node;

    /// \brief Destructor
    ~Characteristic();

    /// \brief Returns true if this topology characteristic belongs to an
    /// irregular (non-quad) face
    bool IsNonQuadPatch() const { return _nonquad; }

    /// \brief Returns the map this characteristic belongs to
    CharacteristicMap const & GetCharacteristicMap() const { return _characteristicMap; }

    /// \brief Evaluate basis functions for position and first derivatives at a
    /// given (s,t) parametric location of a patch.
    ///
    /// @param s        Patch coordinate (in coarse face normalized space)
    ///
    /// @param t        Patch coordinate (in coarse face normalized space)
    ///
    /// @param wP       Weights (evaluated basis functions) for the position
    ///
    /// @param wDs      Weights (evaluated basis functions) for derivative wrt s
    ///
    /// @param wDt      Weights (evaluated basis functions) for derivative wrt t
    ///
    /// @param quadrant Domain quadrant containing (s,t) location (required in
    ///                 order to obtain the correct supports for terminal node
    ///                 sub-patches)
    ///
    /// @param maxLevel Dynamic level of isolation
    ///
    /// @return         The leaf node pointing to the sub-patch evaluated
    ///
    Node EvaluateBasis(float s, float t, float wP[], float wDs[], float wDt[],
            unsigned char * quadrant, short maxLevel=11) const;

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
    struct NodeDescriptor {

        /// \brief Translation constructor
        NodeDescriptor(int value=0) { field0=value; }

        /// \brief Set bitfields for REGULAR / END nodes
        ///
        ///  Field         | Bits | Content
        ///  --------------|:----:|---------------------------------------------------
        ///  type          | 2    | NodeType
        ///  nonquad       | 1    | whether the patch is the child of a non-quad face
        ///  single crease | 1    | Whether the patch is of "single crease" type
        ///  depth         | 4    | level of isolation of the patch
        ///  boundary      | 4    | boundary edge mask encoding
        ///  v             | 10   | log2 value of u parameter at first patch corner
        ///  u             | 10   | log2 value of v parameter at first patch corner
        void SetPatch(unsigned short type, unsigned short nonquad,
            unsigned short singleCrease, unsigned short depth,
                unsigned short boundary, short u, short v);

        /// \brief Set bitfields for RECURSIVE nodes
        ///
        ///  Field         | Bits | Content
        ///  --------------|:----:|---------------------------------------------------
        ///  type          | 2    | NodeType
        ///  nonquad       | 1    | whether the patch is the child of a non-quad face
        ///  depth         | 4    | level of isolation of the patches
        ///  v             | 10   | log2 value of u parameter at first patch corner
        ///  u             | 10   | log2 value of v parameter at first patch corner
        void SetRecursive(unsigned short nonquad,
            unsigned short depth, short u, short v);

        /// \brief Set bitfields for TERMINAL nodes
        ///
        ///  Field         | Bits | Content
        ///  --------------|:----:|---------------------------------------------------
        ///  type          | 2    | NodeType
        ///  nonquad       | 1    | whether the patch is the child of a non-quad face
        ///  unused        | 1    |
        ///  depth         | 4    | level of isolation of the patches
        ///  evIndex       | 4    | local index of the extraordinary vertex
        ///  v             | 10   | log2 value of u parameter at first patch corner
        ///  u             | 10   | log2 value of v parameter at first patch corner
        void SetTerminal(unsigned short nonquad,
            unsigned short depth, unsigned short evIndex, short u, short v);

        /// \brief Resets everything to 0
        void Clear() { field0 = 0; }

        /// \brief Returns the type for the sub-patch.
        NodeType GetType() const { return (NodeType)unpackBitfield(field0, 2, 0); }

        /// \brief Returns the level of subdivision of the sub-patch
        unsigned short GetDepth() const { return (unsigned short)unpackBitfield(field0, 4, 4); }

        /// \brief Returns the boundary edge encoding for the sub-patch.
        unsigned short GetBoundaryMask() const { return (unsigned short)unpackBitfield(field0, 4, 8); }

        /// \brief Returns the number of boundary edges in the sub-patch (-1 for invalid mask)
        unsigned short GetBoundaryCount() const;

        /// \brief Returns local index of the extraordinary vertex in a terminal patch
        unsigned short GetEvIndex() const { return (unsigned short)unpackBitfield(field0, 4, 8); }

        /// \brief True if the parent coarse face is a non-quad
        bool NonQuadRoot() const { return unpackBitfield(field0, 1, 2)!=0; }

        /// \brief Returns true if the patch is of "single crease" type
        bool SingleCrease() const { return unpackBitfield(field0, 1, 3)!=0; }

        /// \brief Returns the log2 value of the u parameter at the top left corner of
        /// the patch
        unsigned short GetU() const { return (unsigned short)unpackBitfield(field0, 10, 12); }

        /// \brief Returns the log2 value of the v parameter at the top left corner of
        /// the patch
        unsigned short GetV() const { return (unsigned short)unpackBitfield(field0, 10, 22); }

        /// \brief Returns the fraction of normalized parametric space covered by the
        /// sub-patch.
        float GetParamFraction() const;

        /// \brief Maps the (u,v) parameterization from coarse to refined
        /// The (u,v) pair is mapped from the coarse face parameterization to
        /// the refined face parameterization
        ///
        void MapCoarseToRefined( float & u, float & v ) const;

        /// \brief Maps the (u,v) parameterization from refined to coarse
        /// The (u,v) pair is mapped from the refined face parameterization to
        /// the coarse face parameterization
        ///
        void MapRefinedToCoarse( float & u, float & v ) const;

        unsigned int field0:32;
    };

    struct Support;

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

        /// \brief Returns a pointer to the characteric that owns this node
        Characteristic const * GetCharacteristic() const {
            return _characteristic;
        }

        /// \brief Returns the number of support points required for the
        /// sub-patch pointed to by this node
        int GetNumSupports(int quadrant, short maxLevel=11) const;

        /// \brief Returns the index of the requested support in the
        /// characteristic.
        /// (supportIndex is relative : in range [0, GetNumSupports()])
        Index GetSupportIndex(int supportIndex, int evIndex, short maxLevel=11) const;

        /// \brief Returns the requested support at index
        /// (supportIndex is relative : in range [0, GetNumSupports()])
        Support GetSupport(int supportIndex, int evIndex, short maxLevel=11) const;

        /// \brief Returns the creased edge sharpness
        /// note : the value is undefined for any node other than REGULAR
        ///        with a SingleCrease flag set to true
        float GetSharpness() const;

    public:

        /// \brief Returns the number of children nodes
        int GetNumChildrenNodes() const;

        /// \brief Returns the node's child at index
        /// note : 4 children for RECURSIVE nodes, 1 child for TERMINAL nodes
        Node GetNodeChild(int childIndex=0) const;

        /// \brief Returns the node's relative offset in the tree (integer stride)
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

        static int getRegularNodeSize(bool isSharp) { return isSharp ? 3 : 2; }

        static int getEndCapNodeSize() { return 2; }

        static int getTerminalNodeSize() { return 3; }

        static int getRecursiveNodeSize() { return 6; }

        static int getNodeSize(NodeType nodeType, bool isSingleCrease);

        Index getFirstSupportIndex() const;

        static int getNumEndCapSupports(EndCapType type);

        static int getNumSupports(NodeType nodeType);

        int getNodeSize() const;

        friend class Characteristic;
        friend class internal::CharacteristicBuilder;
        friend void printCharacteristicTreeNode(FILE * fout, Node node, int charIndex);

        Node(Characteristic const * ch, int treeOffset) :
            _characteristic(ch), _treeOffset(treeOffset) { }

        Characteristic const * _characteristic;
        int _treeOffset;
    };


    /// \brief Returns the size of the patches tree (number of ints)
    int GetTreeSize() const { return (int)_tree.size(); }

    /// \brief Returns the tree data
    int const * GetTreeData() const { return &_tree[0]; }

    /// \brief Returns a pointer to the root node of the sub-patches tree
    Node GetTreeRootNode() const { return Node(this, 0); }

    /// \brief Returns the node corresponding to the sub-patch at the given (s,t) location
    Node GetTreeNode(float s, float t, unsigned char * quadrant, short maxLevel=11) const;

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

    /// \brief Writes a GraphViz 'dot' digraph of the tree
    void WriteTreeDigraph(FILE * fout,
        int charIndex=0, bool showIndices=true, bool isSubgraph=false) const;
    //@}

private:

    friend class internal::CharacteristicBuilder;

    // The sub-patch "tree" is stored as a linear buffer of integers for
    // efficient look-up & traversal on a GPU. Use the Node class to traverse
    // the tree and access each node's data.
    std::vector<int> _tree;

public:

    //
    // Supports
    //

    //@{
    ///  @name Sub-patch support stencils
    ///

    struct Support {

        Support(int _size, LocalIndex const * _indices, float const * _weights) :
            size(_size), indices(_indices), weights(_weights) { }

        short size;
        LocalIndex const * indices;
        float const      * weights;
    };

    /// \brief Returns the total number of supports for this characteristic
    int GetNumSupportsTotal() const { return (int)_sizes.size(); }

    /// \brief Returns the number of supports needing to be evaluated at a given level
    int GetNumSupports(int levelIndex) const { return _numSupports[levelIndex]; }

    /// \brief Returns the support data for the support of given index
    Support GetSupport(Index supportIndex) const;

    /// \brief Returns a vector containing the size of each support
    std::vector<short> const & GetSupportsSizes() const { return _sizes; }

    /// \brief Returns a vector of supports control vertex indices
    std::vector<LocalIndex> const & GetSupportIndices() const { return _indices; }

    /// \brief Returns a vector of the supports stencil weights
    std::vector<int> const & GetSupportWeights() const { return _offsets; }

    /// \brief Returns a vector of the offsets to each support indices & weights
    std::vector<int> const & GetSupportOffsets() const { return _offsets; }

    /// \brief Returns the number of control vertices in the characteristic neighborhood
    short GetNumControlVertices() const { return _numControlVertices; }

    //@}

private:

    int                     _numControlVertices;

    short                   _numSupports[11];

    std::vector<short>      _sizes;
    std::vector<LocalIndex> _indices;
    std::vector<int>        _offsets;
    std::vector<float>      _weights;

public:

    ///
    /// Neighborhoods
    ///

    //@{
    ///  @name Neighborhood access methods
    ///

    /// \brief Returns the number of (rotated) neighborhoods
    int GetNumNeighborhoods() const {
        return (int)_neighborhoods.size();
    }

    /// \brief Returns the neighborhood at 'index'
    Neighborhood const * GetNeighborhood(int index) const {
        return _neighborhoods[index];
    }

    /// \brief Returns the starting edge (rotation) at 'index'
    int GetStartingEdge(int index) const {
        return _startEdges[index];
    }

    /// \brief Returns the index of a neighborhood equivalent to
    /// 'n' (or INDEX_INVALID)
    int FindEquivalentNeighborhood(Neighborhood const & n) const;

    //@}

private:

    bool hasDynamicIsolation() const;

    EndCapType getEndCapType() const;

    void reserveNeighborhoods(int count);

    void addNeighborhood(Neighborhood const * n, int startEdge);

    void shrink_to_fit();

    std::vector<Neighborhood const *> _neighborhoods;
    std::vector<int> _startEdges;

private:

    friend class internal::CharacteristicBuilder;
    friend class CharacteristicMap;
    friend class SubdivisionPlanTable;

    Characteristic(CharacteristicMap const & charmap,
                   int numControlVertices,
                   bool nonquad) :
        _characteristicMap(charmap),
        _numControlVertices(numControlVertices),
        _nonquad(nonquad) { }

    bool _nonquad;

    CharacteristicMap const & _characteristicMap;
};


//
// Inline implementation
//

inline Characteristic::Node
Characteristic::Node::GetNodeChild(int childIndex) const {
    int const * offsetPtr = getNodeData() + 2 + childIndex;
    return Node(_characteristic, *offsetPtr);
}


inline int
Characteristic::Node::getNodeSize(
    Characteristic::NodeType nodeType, bool isSingleCrease) {
    switch (nodeType) {
        case Characteristic::NODE_REGULAR  : return getRegularNodeSize(isSingleCrease);
        case Characteristic::NODE_END      : return getEndCapNodeSize();
        case Characteristic::NODE_TERMINAL : return getTerminalNodeSize();
        case Characteristic::NODE_RECURSIVE: return getRecursiveNodeSize();
    }
    assert(0);
    return 0;
}

inline int
Characteristic::Node::getNodeSize() const {
    NodeDescriptor desc = GetDescriptor();
    return getNodeSize(desc.GetType(), desc.SingleCrease());
}

inline int
Characteristic::Node::getNumEndCapSupports(EndCapType type) {
    switch (type) {
        case ENDCAP_BSPLINE_BASIS: return 16;
        case ENDCAP_GREGORY_BASIS: return 20;
        default:
            assert(0);
    }
    return 0;
}

inline int
Characteristic::Node::getNumSupports(Characteristic::NodeType nodeType) {
    switch (nodeType) {
        case Characteristic::NODE_REGULAR  : return 16;
        case Characteristic::NODE_TERMINAL : return 25;
        case Characteristic::NODE_RECURSIVE: return 0;
    }
    assert(0);
    return 0;
}


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


inline void
Characteristic::NodeDescriptor::SetPatch(unsigned short type,
    unsigned short nonquad, unsigned short singleCrease, unsigned short depth,
        unsigned short boundary, short u, short v) {
    assert(type==NODE_REGULAR||type==NODE_END);
    field0 = packBitfield(v,           10, 22) |
             packBitfield(u,           10, 12) |
             packBitfield(boundary,     4,  8) |
             packBitfield(depth,        4,  4) |
             packBitfield(singleCrease, 1,  3) |
             packBitfield(nonquad,      1,  2) |
             packBitfield(type,         2,  0);
}

inline void
Characteristic::NodeDescriptor::SetRecursive(unsigned short nonquad,
    unsigned short depth, short u, short v) {
    field0 = packBitfield(v,             10, 22) |
             packBitfield(u,             10, 12) |
             packBitfield(depth,          4,  4) |
             packBitfield(nonquad,        1,  2) |
             packBitfield(NODE_RECURSIVE, 2,  0);
}

inline void
Characteristic::NodeDescriptor::SetTerminal(unsigned short nonquad,
    unsigned short depth, unsigned short evIndex, short u, short v) {
    field0 = packBitfield(v,             10, 22) |
             packBitfield(u,             10, 12) |
             packBitfield(evIndex,        4,  8) |
             packBitfield(depth,          4,  4) |
             packBitfield(nonquad,        1,  2) |
             packBitfield(NODE_TERMINAL,  2,  0);
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
Characteristic::NodeDescriptor::MapCoarseToRefined( float & u, float & v ) const {

    float frac = GetParamFraction(),
          pu = (float)GetU()*frac,
          pv = (float)GetV()*frac;

    u = (u - pu) / frac,
    v = (v - pv) / frac;
}

inline void
Characteristic::NodeDescriptor::MapRefinedToCoarse( float & u, float & v ) const {

    float frac = GetParamFraction(),
          pu = (float)GetU()*frac,
          pv = (float)GetV()*frac;

    u = u * frac + pu,
    v = v * frac + pv;
}

inline Characteristic::Support
Characteristic::GetSupport(Index supportIndex) const {
    assert(!_offsets.empty() && supportIndex<_offsets.size());
    int size = _sizes[supportIndex];
    Index offset = _offsets[supportIndex];
    return Support(size, &_indices[offset], &_weights[offset]);
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_CHARACTERISTIC_H */
