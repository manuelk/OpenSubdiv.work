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

#ifndef OPENSUBDIV3_FAR_CHARACTERISTIC_TREE_BUILDER_H
#define OPENSUBDIV3_FAR_CHARACTERISTIC_TREE_BUILDER_H

#include "../version.h"

#include "../far/types.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

struct EndCapBuilder;
struct PatchFaceTag;
class StencilTable;
class TopologyRefiner;
class Characteristic;
class CharacteristicMap;

// A specialized builder for subdivision plan hierarchies
class CharacteristicBuilder {

public:

    // Constructor
    CharacteristicBuilder(
        TopologyRefiner const & refiner, CharacteristicMap const & charmap);

    ~CharacteristicBuilder();

    // Creats a characteristic for the given level & face
    Characteristic const * Create(int levelIndex, int faceIndex);

    // Perform final operations on stencils and returns the tables
    // Note : should not be called until all trees have been written.
    StencilTable const * FinalizeStencils();

    StencilTable const * FinalizeVaryingStencils();

private:

    int getTerminalNodeSize(int levelIndex) const;

    int getRecursiveNodeSize(int levelIndex, int faceIndex) const;

    int getEndCapNodeSize() const;

    int getRegularNodeSize(int levelIndex, int faceIndex) const;

    int getNodeSize(int levelIndex, int faceIndex) const;

    int getTreeSize(int levelIndex, int faceIndex) const;


    bool nodeIsTerminal(int levelIndex, int faceIndex, int * evIndex=0) const;

    int writeTerminalNode(int leveIndex, int faceIndex, int evIndex, int offset, uint8_t * data) const;

    int writeRecursiveNode(int leveIndex, int faceIndex, int offset, uint8_t * data) const;

    int writeRegularNode(int leveIndex, int faceIndex, uint8_t * data) const;

    int writeEndCapNode(int leveIndex, int faceIndex, uint8_t * data) const;

    int writeNode(int leveIndex, int faceIndex, int offset, uint8_t * data) const;

    void writeTree(int levelIndex, int faceIndex, int * treePtr) const;

    bool computeSubPatchDomain(int levelIndex, Index faceIndex, short * s, short * t) const;

private:

    //
    // General state
    //

    TopologyRefiner const & _refiner;

    CharacteristicMap const & _charmap;

    StencilTable const * _stencils;

    std::vector<PatchFaceTag> _patchTags;

    //
    // End-cap stencils
    //

    EndCapBuilder * _endcapBuilder;

    //
    // Misc. subdivision level offsets
    //

    std::vector<PatchFaceTag const *> _levelPatchTags;
    std::vector<Index> _levelVertOffsets;
};


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_CHARACTERISTIC_TREE_BUILDER_H */

