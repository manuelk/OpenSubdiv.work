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

#ifndef OPENSUBDIV3_FAR_CHARACTERISTIC_FACTORY_H
#define OPENSUBDIV3_FAR_CHARACTERISTIC_FACTORY_H


#include "../version.h"

#include "../far/types.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

struct PatchFaceTag;
class TopologyRefiner;
class StencilTable;

typedef std::vector<PatchFaceTag> PatchFaceTagVector;

class Characteristic {

public:


    //
    // Tree
    //

    //@{
    ///  @name Tree access methods
    ///

    ///
    ///  Field      | size | Content
    ///  -----------|:----:|----------------------------------------------------
    /// Regular node layout
    ///  header     | 1    | see NodeDescriptor
    ///  sharpness  | 1    | crease sharpness (single crease nodes only)
    ///  supports   | 16   | indices
    ///
    /// End node layout:
    ///  header     | 1    | see NodeDescriptor
    ///  supports   | 16   | indices
    ///
    /// Recursive node layout:
    ///  header     | 1    | see NodeDescriptor
    ///  offsets    | 4    | offsets to 4 children nodes
    ///
    /// Terminal node layout:
    ///  header     | 1    | see NodeDescriptor
    ///  offsets    | 4    | offsets to children nodes
    ///  supports   | 25   | indices
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
    ///  depth        | 4    | the subdivision level of the node
    ///  transition   | 4    | transition edge mask encoding
    ///  boundary     | 4    | boundary edge mask encoding
    ///  singleCrease | 1    | true if "single crease" patch
    ///
    struct NodeDescriptor {

        /// \brief Translation constructor
        NodeDescriptor(int value) { field0=value; }

        void Set(unsigned short type, unsigned short depth, unsigned short boundary,
            unsigned short transition, unsigned short singleCrease=false) {
            field0 = ((singleCrease ? 1:0) << 14) |
                     ((boundary & 0xf)     << 10) |
                     ((transition & 0xf)   <<  6) |
                     ((depth & 0xf)        <<  2) |
                     (type & 0x3);
        }

        NodeDescriptor & operator=(int value) { field0 = value; return *this; }

        /// \brief Resets everything to 0
        void Clear() { field0 = 0; }

        /// \brief Returns the type for the patch.
        NodeType GetType() const { return (NodeType)(field0 & 0x3); }

        /// \brief Returns the level of subdivision of the patch
        unsigned short GetDepth() const { return  (unsigned short)((field0 >> 2) & 0xf); }

        /// \brief Returns the transition edge encoding for the patch.
        unsigned short GetTransition() const { return (unsigned short)((field0 >> 6) & 0xf); }

        /// \brief Returns the boundary edge encoding for the patch.
        unsigned short GetBoundary() const { return (unsigned short)((field0 >> 10) & 0xf); }

        /// \brief Returns true if the patch is of "single crease" type
        bool IsSingleCrease() const { return (field0 >> 14) & 0x1; }

        int field0:32;
    };

    /// Tree Node
    class Node {

    public:

        /// \brief Returns the node descriptor
        NodeDescriptor GetDescriptor() const { 
            return _characteristic->_tree[_treeOffset]; 
        }

        /// \brief Returns a pointer to the indices of the support points
        Index const * GetSupportIndices() const;

        Node GetChildNode(int childIndex=0) const;

        Characteristic const * GetCharacteristic() const { return _characteristic; }

        int GetTreeOffset() const { return _treeOffset; }

        bool operator == (Node const & other) const {
            return _characteristic == other._characteristic &&
                _treeOffset == other._treeOffset;
        }


    private:

        int const * getNodeData() const {
            return &_characteristic->_tree[_treeOffset];
        }

        friend class Characteristic;

        Node(Characteristic const * ch, int treeOffset) :
            _characteristic(ch), _treeOffset(treeOffset) { }

        Characteristic const * _characteristic;
        int _treeOffset;
    };


    /// \brief Returns the size (in bytes) of the patches tree
    int GetTreeSize() const { return _treeSize; }

    /// \brief Returns a pointer to the root node of the sub-patches tree
    Node GetTreeRootNode() const { return Node(this, 0); }

    /// \brief Returns a the node corresponding to the sub-patch at the given (s,t) location
    Node GetNode(float s, float t) const;
    //@}



public:

    //@{
    ///  @name Evaluation methods
    ///

    /// \brief Evaluate basis functions for position and first derivatives at a
    /// given (s,t) parametric location of a patch.
    ///
    /// @param handle  A patch handle indentifying the sub-patch containing the
    ///                (s,t) location
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
    void EvaluateBasis(Node n, float s, float t, float wP[], float wDs[], float wDt[]) const;

    //@}


private:

    friend class CharacteristicBuilder;

    int * _tree,
          _treeSize;
};


class CharacteristicMap {

public:

    enum EndCapType {
        ENDCAP_NONE = 0,             ///< no endcap
        ENDCAP_BILINEAR_BASIS,       ///< use bilinear quads (4 cp) as end-caps
        ENDCAP_BSPLINE_BASIS,        ///< use BSpline basis patches (16 cp) as end-caps
        ENDCAP_GREGORY_BASIS,        ///< use Gregory basis patches (20 cp) as end-caps
    };


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

class CharacteristicMapFactory {

public:

    typedef CharacteristicMap::EndCapType EndCapType;

    struct Options {

        Options() :
             endCapType(CharacteristicMap::ENDCAP_BSPLINE_BASIS),
             useTerminalNodes(false) { }

        /// \brief Get endcap patch type
        EndCapType GetEndCapType() const { return (EndCapType)endCapType; }

        /// \brief Set endcap patch type
        void SetEndCapType(EndCapType e) { endCapType = e; }

        unsigned int endCapType       : 3, ///< Type of end-cap patches
                     useTerminalNodes : 1; ///< Use "terminal" nodes on patches with single EV
    };

    static CharacteristicMap const * Create(TopologyRefiner const & refiner,
        PatchFaceTagVector const & patchTags,
           Options options=Options());

private:

};

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_CHARACTERISTIC_FACTORY_H */

