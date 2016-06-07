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

class TopologyRefiner;
struct PatchFaceTag;
typedef std::vector<PatchFaceTag> PatchFaceTagVector;

struct Characteristic {


    //
    // Tree
    //

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

    /// ModeDescriptor
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

        void Set(unsigned short type, unsigned short depth, unsigned short boundary,
            unsigned short transition, unsigned short singleCrease=false) {
            field0 = ((singleCrease ? 1:0) << 14) |
                     ((boundary & 0xf)     << 10) |
                     ((transition & 0xf)   <<  6) |
                     ((depth & 0xf)        <<  2) |
                     (type & 0x3);
        }

        /// \brief Resets everything to 0
        void Clear() { field0 = 0; }

        /// \brief Returns the type for the patch.
        NodeType GetType() const { return (NodeType)(field0 & 0x3); }

        /// \brief Returns the transition edge encoding for the patch.
        unsigned short GetTransition() const { return (unsigned short)((field0 >> 6) & 0xf); }

        /// \brief Returns the boundary edge encoding for the patch.
        unsigned short GetBoundary() const { return (unsigned short)((field0 >> 10) & 0xf); }

        /// \brief Returns true if the patch is of "single crease" type
        bool IsSingleCrease() const { return (field0 >> 14) & 0x1; }

        unsigned int field0:32;
    };

    int * tree,
          treeSize;
};


class CharacteristicFactory {

public:

    struct Options {

        enum EndCapType {
            ENDCAP_NONE = 0,             ///< no endcap
            ENDCAP_BILINEAR_BASIS,       ///< use bilinear quads (4 cp) as end-caps
            ENDCAP_BSPLINE_BASIS,        ///< use BSpline basis patches (16 cp) as end-caps
            ENDCAP_GREGORY_BASIS,        ///< use Gregory basis patches (20 cp) as end-caps
        };

        Options() :
             endCapType(ENDCAP_BSPLINE_BASIS) { }

        /// \brief Get endcap patch type
        EndCapType GetEndCapType() const { return (EndCapType)endCapType; }

        /// \brief Set endcap patch type
        void SetEndCapType(EndCapType e) { endCapType = e; }

        unsigned int endCapType : 3; ///< EndCapType
    };

    static Characteristic const * Create(TopologyRefiner const & refiner,
        PatchFaceTagVector const & patchTags,
           Options options=Options());

private:

};

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_CHARACTERISTIC_FACTORY_H */

