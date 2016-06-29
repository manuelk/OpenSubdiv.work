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
#include "../far/characteristicTreeBuilder.h"
#include "../far/endCapBSplineBasisPatchFactory.h"
#include "../far/endCapGregoryBasisPatchFactory.h"
#include "../far/neighborhoodBuilder.h"
#include "../far/patchFaceTag.h"
#include "../far/topologyRefiner.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {


//
// Characteristic factory
//

Index
CharacteristicMapFactory::findOrAddCharacteristic(TopologyRefiner const & refiner,
    NeighborhoodBuilder & neighborhoodBuilder, CharacteristicTreeBuilder & treeBuilder,
         int faceIndex, CharacteristicMap * charmap) {

    TopologyLevel const & coarseLevel = refiner.GetLevel(0);

    Neighborhood const * n = neighborhoodBuilder.Create(coarseLevel, faceIndex);

    int rotation = 0;
    Index charIndex = charmap->findCharacteristic(*n, &rotation);

    if (charIndex==INDEX_INVALID) {

        // topological configuration does not exist in the map : create a new
        // characteristic & add to the map

        charIndex = charmap->GetNumCharacteristics();
        assert(charIndex!=INDEX_INVALID);

        int valence = n->GetValence(),
            regValence = Sdc::SchemeTypeTraits::GetRegularFaceSize(refiner.GetSchemeType());

        if (valence!=regValence) {
            ConstIndexArray childFaces = coarseLevel.GetFaceChildFaces(faceIndex);
            for (int i=0; i<valence; ++i) {
                Characteristic * ch = new Characteristic(charmap);
                ch->writeCharacteristicTree(treeBuilder, 1, childFaces[i]);
                charmap->_characteristics.push_back(ch);
            }
        } else {
            Characteristic * ch = new Characteristic(charmap);
            ch->writeCharacteristicTree(treeBuilder, 0, faceIndex);
            charmap->_characteristics.push_back(ch);
        }

        charmap->addCharacteristicToHash(
            coarseLevel, neighborhoodBuilder, faceIndex, charIndex, valence);
    }
    delete n;
    return charIndex;
}

CharacteristicMap const *
CharacteristicMapFactory::Create(TopologyRefiner const & refiner, Options options) {

    // XXXX we do not support those end-cap types yet
    if (options.GetEndCapType()==ENDCAP_BILINEAR_BASIS ||
        options.GetEndCapType()==ENDCAP_LEGACY_GREGORY) {
        return nullptr;
    }

    // identify patch types
    Far::PatchFaceTagVector patchTags;
    Far::PatchFaceTag::IdentifyAdaptivePatches(
        refiner, patchTags, refiner.GetNumLevels(), options.useSingleCreasePatch);


    CharacteristicTreeBuilder treesBuilder(refiner,
        patchTags, options.maxIsolationLevel, options.GetEndCapType(), options.useTerminalNode);

    TopologyLevel const & coarseLevel = refiner.GetLevel(0);

    int nfaces = coarseLevel.GetNumFaces();

    CharacteristicMap * charmap = new CharacteristicMap(options);

    if (options.hashSize>0) {

        // hash topology : faces with redundant topological configurations share
        // the same characteristic

        // XXXX manuelk TODO this can only work with localized stencils for
        // support verts. Right now, characteristic trees only gather global
        // stencil indices.

        charmap->_characteristicsHash.resize(options.hashSize, INDEX_INVALID);

        NeighborhoodBuilder neighborhoodBuilder;

        for (int face = 0; face < nfaces; ++face) {

            if (coarseLevel.IsFaceHole(face)) {
                continue;
            }

            findOrAddCharacteristic(refiner, neighborhoodBuilder, treesBuilder, face, charmap);
        }
    } else {

        // hash map size set to 0 : each face gets its own characteristic

        int regFaceSize = Sdc::SchemeTypeTraits::GetRegularFaceSize(refiner.GetSchemeType()),
            nchars = 0;

        // Count the number of characteristics (non-quads have more than 1)
        for (int face = 0; face < nfaces; ++face) {
            if (coarseLevel.IsFaceHole(face)) {
                continue;
            }
            ConstIndexArray fverts = coarseLevel.GetFaceVertices(face);
            nchars += fverts.size()==regFaceSize ? 1 : fverts.size();
        }

        // Allocate & write the characteristics
        charmap->_characteristics.reserve(nchars);

        for (int face = 0; face < coarseLevel.GetNumFaces(); ++face) {

            if (coarseLevel.IsFaceHole(face)) {
                continue;
            }

            ConstIndexArray verts = coarseLevel.GetFaceVertices(face);

            if (verts.size()==regFaceSize) {

                Characteristic * ch = new Characteristic(charmap);
                ch->writeCharacteristicTree(treesBuilder, 0, face);

                charmap->_characteristics.push_back(ch);            
            } else {
                ConstIndexArray children = coarseLevel.GetFaceChildFaces(face);
                for (int i=0; i<children.size(); ++i) {

                    Characteristic * ch = new Characteristic(charmap);
                    ch->writeCharacteristicTree(treesBuilder, 1, children[i]);

                    charmap->_characteristics.push_back(ch);            
                }
            }
        }
    }

    charmap->_localPointStencils = treesBuilder.FinalizeStencils();
    charmap->_localPointVaryingStencils = treesBuilder.FinalizeVaryingStencils();

    return charmap;
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

