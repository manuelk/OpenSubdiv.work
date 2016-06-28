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

#include "../far/characteristicMapFactory.h"
#include "../far/characteristicMap.h"
#include "../far/characteristicTreeBuilder.h"
#include "../far/endCapBSplineBasisPatchFactory.h"
#include "../far/endCapGregoryBasisPatchFactory.h"
#include "../far/patchFaceTag.h"
#include "../far/topologyRefinerFactory.h"
#include "../vtr/refinement.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {


//
// Characteristic factory
//

void
CharacteristicMapFactory::writeCharacteristicTree(
    CharacteristicTreeBuilder & builder, int levelIndex, int faceIndex, Characteristic * ch) {

    int treeSize = builder.GetTreeSize(levelIndex, faceIndex);

    int * treePtr = new int[treeSize];

#if 0
    // debug : paint memory
    for (int i=0; i<treeSize; ++i) {
        treePtr[i]=-1;
    }
#endif

    builder.WriteTree(levelIndex, faceIndex, treePtr);

    ch->_treeSize = treeSize;
    ch->_tree = treePtr;
}

CharacteristicMap const *
CharacteristicMapFactory::Create(TopologyRefiner const & refiner,
    PatchFaceTagVector const & patchTags, Options options) {

    // XXXX we do not support those end-cap types yet
    if (options.GetEndCapType()==ENDCAP_BILINEAR_BASIS ||
        options.GetEndCapType()==ENDCAP_LEGACY_GREGORY) {
        return nullptr;
    }

    CharacteristicTreeBuilder builder(refiner,
        patchTags, options.GetEndCapType(), options.useTerminalNodes);

    TopologyLevel const & coarseLevel = refiner.GetLevel(0);

    int regFaceSize =
        Sdc::SchemeTypeTraits::GetRegularFaceSize(refiner.GetSchemeType());

    int nfaces = coarseLevel.GetNumFaces(),
        nchars = 0;

    // Count the number of characteristics (non-quads have more than 1)
    for (int face = 0; face < nfaces; ++face) {
        if (coarseLevel.IsFaceHole(face)) {
            continue;
        }
        ConstIndexArray fverts = coarseLevel.GetFaceVertices(face);
        nchars += fverts.size()==regFaceSize ? 1 : fverts.size();
    }

    CharacteristicMap * charmap =
        new CharacteristicMap(options.GetEndCapType());

    // Allocate & write the characteristics
    charmap->_characteristics.reserve(nchars);

    for (int face = 0; face < coarseLevel.GetNumFaces(); ++face) {

        if (coarseLevel.IsFaceHole(face)) {
            continue;
        }

        ConstIndexArray verts = coarseLevel.GetFaceVertices(face);

        if (verts.size()==regFaceSize) {

            Characteristic * ch = new Characteristic;
            ch->_characteristicMap = charmap;

            writeCharacteristicTree(builder, 0, face, ch);

            charmap->_characteristics.push_back(ch);            
        } else {
            ConstIndexArray children = coarseLevel.GetFaceChildFaces(face);
            for (int i=0; i<children.size(); ++i) {

                Characteristic * ch = new Characteristic;
                ch->_characteristicMap = charmap;

                writeCharacteristicTree(builder, 1, children[i], ch);

                charmap->_characteristics.push_back(ch);            
            }
        }
    }

    charmap->_localPointStencils = builder.FinalizeStencils();
    charmap->_localPointVaryingStencils = builder.FinalizeVaryingStencils();

    return charmap;
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

