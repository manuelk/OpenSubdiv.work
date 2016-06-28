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

void
writeCharacteristicTree(CharacteristicTreeBuilder & builder,
    int levelIndex, int faceIndex, int * treeSize, int ** treePtr) {

    assert(treeSize && treePtr);

    int size = builder.GetTreeSize(levelIndex, faceIndex);
    int * ptr = new int[size];

#if 0
    // debug : paint memory
    for (int i=0; i<treeSize; ++i) {
        ptr[i]=-1;
    }
#endif

    builder.WriteTree(levelIndex, faceIndex, ptr);

    *treeSize = size;
    *treePtr = ptr;
}

Index
CharacteristicMapFactory::findOrAddCharacteristic(TopologyRefiner const & refiner,
    NeighborhoodBuilder & neighborhoodBuilder, CharacteristicTreeBuilder & treeBuilder,
         int faceIndex, CharacteristicMap * charmap) {

    TopologyLevel const & coarseLevel = refiner.GetLevel(0);

    Neighborhood const * n = neighborhoodBuilder.Create(coarseLevel, faceIndex);

    Index charIndex = INDEX_INVALID;

    int hash = n->GetHash(),
        hashCount = (int)charmap->_characteristicsHash.size(),
        rotation = 0;

    for (int i=0; i<hashCount; ++i) {

        int hashIndex = (hash + i) % hashCount;

        charIndex = charmap->_characteristicsHash[hashIndex];
        if (charIndex==INDEX_INVALID) {
            charIndex = charmap->GetNumCharacteristics();
            break;
        }

        Characteristic const * ch = charmap->_characteristics[charIndex];
        for (int j=0; j<ch->GetNumNeighborhoods(); ++j) {
            if (n->IsEquivalent(*ch->GetNeighborhood(j))) {
                rotation = ch->GetStartingEdge(j);
                return charIndex;
            }
        }
    }

    assert(charIndex!=INDEX_INVALID);

    int valence = n->GetValence(),
        regValence = Sdc::SchemeTypeTraits::GetRegularFaceSize(refiner.GetSchemeType());

    if (valence!=regValence) {
        ConstIndexArray childFaces = coarseLevel.GetFaceChildFaces(faceIndex);
        for (int i=0; i<valence; ++i) {
            Characteristic * ch = new Characteristic(charmap);
            writeCharacteristicTree(treeBuilder, 1, childFaces[i],  &ch->_treeSize, &ch->_tree);
            charmap->_characteristics.push_back(ch);
        }
    } else {
        Characteristic * ch = new Characteristic(charmap);
        writeCharacteristicTree(treeBuilder, 0, faceIndex, &ch->_treeSize, &ch->_tree);
        charmap->_characteristics.push_back(ch);
    }

    charmap->addCharacteristicToHash(
        coarseLevel, neighborhoodBuilder, faceIndex, charIndex, valence);

    delete n;

    return charIndex;
}

CharacteristicMap const *
CharacteristicMapFactory::Create(TopologyRefiner const & refiner,
    PatchFaceTagVector const & patchTags, Options options) {

    // XXXX we do not support those end-cap types yet
    if (options.GetEndCapType()==ENDCAP_BILINEAR_BASIS ||
        options.GetEndCapType()==ENDCAP_LEGACY_GREGORY) {
        return nullptr;
    }

    CharacteristicTreeBuilder treesBuilder(refiner,
        patchTags, options.GetEndCapType(), options.useTerminalNodes);

    TopologyLevel const & coarseLevel = refiner.GetLevel(0);

    int nfaces = coarseLevel.GetNumFaces();

    CharacteristicMap * charmap =
        new CharacteristicMap(options.GetEndCapType());

//#define DO_HASH
#ifdef DO_HASH
    charmap->_characteristicsHash.resize(options.hashSize, INDEX_INVALID);

    NeighborhoodBuilder neighborhoodBuilder;

    for (int face = 0; face < nfaces; ++face) {

        if (coarseLevel.IsFaceHole(face)) {
            continue;
        }

        findOrAddCharacteristic(refiner, neighborhoodBuilder, treesBuilder, face, charmap);
    }

#else

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
            writeCharacteristicTree(treesBuilder, 0, face, &ch->_treeSize, &ch->_tree);

            charmap->_characteristics.push_back(ch);            
        } else {
            ConstIndexArray children = coarseLevel.GetFaceChildFaces(face);
            for (int i=0; i<children.size(); ++i) {

                Characteristic * ch = new Characteristic(charmap);
                writeCharacteristicTree(treesBuilder, 1, children[i],  &ch->_treeSize, &ch->_tree);

                charmap->_characteristics.push_back(ch);            
            }
        }
    }
#endif

    charmap->_localPointStencils = treesBuilder.FinalizeStencils();
    charmap->_localPointVaryingStencils = treesBuilder.FinalizeVaryingStencils();

    return charmap;
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

