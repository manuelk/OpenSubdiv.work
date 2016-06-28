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
#include "../far/neighborhoodBuilder.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

void
CharacteristicMap::addCharacteristicToHash(
    TopologyLevel const & level, NeighborhoodBuilder & neighborhoodBuilder,
        int faceIndex, int charIndex, int valence) {

    Characteristic * ch = _characteristics[charIndex];

    ch->reserveNeighborhoods(valence);

    for (int i=0; i<valence; ++i) {

        Neighborhood const * n = neighborhoodBuilder.Create(level, faceIndex, i);

        if (ch->FindEquivalentNeighborhood(*n)!=INDEX_INVALID) {
            continue;
        }

        unsigned int hash = n->GetHash(),
                     hashCount = (unsigned int)_characteristicsHash.size();

        for (unsigned int j=0; j<hashCount; ++j) {

            unsigned int hashIndex = (hash+j) % hashCount;

            assert(hashIndex<hashCount);

            int existingCharIndex = _characteristicsHash[hashIndex];

            if (existingCharIndex == INDEX_INVALID) {
                _characteristicsHash[hashIndex] = charIndex;
                break;
            }
        }

        // XXXX Wade says that startEdge reversing probably counters bug in
        // fastsubdiv code : we probably shouldn't be doing this...
        int startEdge = (valence-i) % valence; 
        ch->addNeighborhood(n, startEdge);

        // XXXX if (macro patches) break;
    }

    // XXXX is this really necessary ?
    ch->shrink_to_fit();
}


void
CharacteristicMap::WriteCharacteristicsDiagraphs(FILE * fout, bool showIndices) const {

    fprintf(fout, "digraph {\n");
    for (int charIndex=0; charIndex<GetNumCharacteristics(); ++charIndex) {

        Characteristic const * ch = GetCharacteristic(charIndex);
        if (ch) {
            ch->WriteTreeDiagraph(fout, charIndex, showIndices, /*isSubgraph*/ true);
        }
    }
    fprintf(fout, "}\n");
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

