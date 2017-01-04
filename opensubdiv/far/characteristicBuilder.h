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

#ifndef OPENSUBDIV3_FAR_CHARACTERISTIC_BUILDER_H
#define OPENSUBDIV3_FAR_CHARACTERISTIC_BUILDER_H

#include "../version.h"

#include "../far/characteristicMap.h"
#include "../far/patchFaceTag.h"
#include "../far/topologyRefiner.h"
#include "../far/types.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class Neighborhood;
class StencilTable;
class Characteristic;

namespace internal {

struct EndCapBuilder;

//
// A specialized builder for subdivision plan hierarchies
//
// The builder API is currently constrained by 2 problems:
//   - stencil table factories cannot operate on a localized neighborhood
//     of the source mesh
//   - the end-cap builders similarly need to first gather all end-caps
//     on the mesh before executing a "finalize" process
// Because of these limitations, we are forced to generate many redundant
// stencils for a given mesh. We also have to keep a list of all the
// characteristics that a mesh adds to the characteristic map, so that we can
// apply the "finalize" step and release transient resources.
//
// With more agile stencil and end-cap builders we will be able to remove
// the FinalizeSupportStencils() call from this API, along with the attending
// garbage collection.
//
class CharacteristicBuilder {

public:

    // Constructor
    CharacteristicBuilder(
        TopologyRefiner const & refiner, CharacteristicMap const & charmap);

    // Destructor
    ~CharacteristicBuilder();

    // Returns a new characteristic for the given topological neighborhood
    // note : the characteristic will not be functional until
    //        FinalizeSupportStencils() has been called
    Characteristic const * Create(
        Index levelIndex, Index faceIndex, Neighborhood const * neighborhood);

    // Finalizes the characteristics created by this builder
    int FinalizeSupportStencils();

private:

    //
    // Proto Nodes
    //

    struct ProtoNode {

        Index faceIndex;       // index of face in Vtr::level

        PatchFaceTag patchTag; // patch tags for the face

        int treeOffset,        // linear node offset in characteristic tree
            firstSupport;      // index of first support point for the node

        unsigned int active      : 1,  // skip inactive nodes
                     levelIndex  : 4,  // index of Vtr::level
                     nodeType    : 2,  // Characteristic::NodeType
                     evIndex     : 2,  // index of xordinary vertex for terminal nodes
                     numChildren : 3;  // number of children linked to the node

        int children[4];       // indices of child nodes in the node-store
    };

    typedef std::vector<ProtoNode> ProtoNodeVector ;

    // store of Proto Nodes sorted by level
    static int const numLevelMax = 11;
    ProtoNodeVector _nodeStore[numLevelMax];

    // reset the store before starting a new characteristic tree
    void resetNodeStore();

    // returns a child of the node (undetermined if child does not exist !)
    ProtoNode & getNodeChild(ProtoNode const & node, short childIndex);

    // returns a const child of the node (undetermined if child does not exist !)
    ProtoNode const & getNodeChild(ProtoNode const & node, short childIndex) const;

    // recursively traverses the children of a refiner face & adds ProtoNodes to the store
    int identifyNode(int levelIndex, Index faceIndex);

    // returns true if a ProtoNode can be converted to a Terminal node
    bool nodeIsTerminal(ProtoNode const & pn, int * evIndex) const;

    // optimizes the node store by converting Recursive nodes into Terminal nodes
    void identifyTerminalNodes();

    // sequentially assigns final value to proto-nodes treeOffset & firstSupport
    void computeNodeOffsets(int * treeSizeOut, short * numSupportsOut, int * numSupportsTotalOut);

    // populates the serialized tree & supports indices buffers
    void populateNodes(int * treePtr, Index * supportsPtr) const;

    // serializes a Regular proto-node
    void populateRegularNode(ProtoNode const & pn, int * treePtr, Index * supportsPtr) const;

    // serializes an End proto-node
    void populateEndCapNode(ProtoNode const & pn, int * treePtr, Index * supportsPtr) const;

    // serializes a terminal proto-node
    void populateTerminalNode(ProtoNode const & pn, int * treePtr, Index * supportsPtr) const;

    // serializes a recursive proto-node
    void populateRecursiveNode(ProtoNode const & pn, int * treePtr, Index * supportsPtr) const;

    // returns the numbers of supports required for a given sub-patch node
    int computeNumSupports(Characteristic::NodeType nodeType, bool useDynamicIsolation) const;

private:

    //
    // End Caps
    //

    EndCapBuilder * _endcapBuilder;

    EndCapType getEndCapType() const { return _charmap.GetOptions().GetEndCapType(); }

private:

    //
    // Build Contexts
    //

    // The BuildContext collects information between the call to Create() and
    // FinalizeSupportStencils(), where it is destroyed.
    struct BuildContext {

        Characteristic * characteristic;   // characteristic created by the builder

        Neighborhood const * neighborhood; // remaps control verts indices

        int levelIndex,       // level of control face (0 for regular, 1 for non-quad)
            faceIndex,        // Vtr::level index of control face
            numSupportsTotal; // total number of supports for the characteristic

        std::vector<Index> supportIndices;  //  stencil indices of supports
    };

    std::vector<BuildContext *> _buildContexts;

private:

    // misc. helpers

    short getMaxIsolationLevel() const { return _refiner.GetAdaptiveOptions().isolationLevel; }

    bool useTerminalNodes() const { return _charmap.GetOptions().useTerminalNode; }

    bool useDynamicIsolation() const { return _charmap.GetOptions().useDynamicIsolation; }

    bool useSingleCreasePatches() const { return _refiner.GetAdaptiveOptions().useSingleCreasePatch; }

    bool computeSubPatchDomain(int levelIndex, Index faceIndex, short * s, short * t) const;

private:

    TopologyRefiner const & _refiner;

    CharacteristicMap const & _charmap;

    std::vector<Index> _levelVertOffsets;
};

} // end namespace internal
} // end namespace Far
} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_CHARACTERISTIC_BUILDER_H */
