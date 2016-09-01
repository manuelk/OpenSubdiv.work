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
#include "../far/characteristicBuilder.h"
#include "../far/error.h"
#include "../far/neighborhoodBuilder.h"
#include "../far/patchFaceTag.h"
#include "../far/subdivisionPlanTable.h"
#include "../far/topologyRefiner.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

void
CharacteristicMap::addCharacteristicToHash(
    TopologyLevel const & level, NeighborhoodBuilder & neighborhoodBuilder,
        int faceIndex, int charIndex, int valence) {

    Characteristic * ch = (Characteristic *)_characteristics[charIndex];

    ch->reserveNeighborhoods(valence);

    for (int i=0; i<valence; ++i) {

        // XXXX manuelk we could probably save a few bytes by adding an option
        // to not generate the remaps for these neighborhoods
        Neighborhood const * neighborhood =
            neighborhoodBuilder.Create(level, faceIndex, i);

        if (ch->FindEquivalentNeighborhood(*neighborhood)!=INDEX_INVALID) {
            continue;
        }

        unsigned int hash = neighborhood->GetHash(),
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
        ch->addNeighborhood(neighborhood, startEdge);

        // XXXX if (macro patches) break;
    }

    // XXXX is this really necessary ?
    ch->shrink_to_fit();
}

Index
CharacteristicMap::findCharacteristic(
    Neighborhood const & neighborhood, int * rotation) const {

    unsigned int hash = neighborhood.GetHash();

    int hashCount = (int)_characteristicsHash.size(),
        charIndex = INDEX_INVALID;

    for (int i=0; i<hashCount; ++i) {

        int hashIndex = (hash + i) % hashCount;

        charIndex = _characteristicsHash[hashIndex];
        if (charIndex==INDEX_INVALID) {
                return charIndex;
        }

        Characteristic const * ch = _characteristics[charIndex];
        for (int j=0; j<ch->GetNumNeighborhoods(); ++j) {
            if (neighborhood.IsEquivalent(*ch->GetNeighborhood(j))) {
                if (rotation) {
                    *rotation = ch->GetStartingEdge(j);
                }
                return charIndex;
            }
        }
    }
    return charIndex;
}

bool
CharacteristicMap::supportsEndCaps(EndCapType type) {

    // XXXX we do not support those end-cap types yet
    if (type==ENDCAP_BILINEAR_BASIS || type==ENDCAP_LEGACY_GREGORY) {
        Error(FAR_CODING_ERROR, "Failure in CharacteristicMap::MapTopology() -- "
            "unsupported EndCapType");
        return false;
    }
    return true;
}

CharacteristicMap::CharacteristicMap(Options options) :
    _options(options), _numMaxSupports(0) {

    _characteristicsHash.resize(_options.hashSize, INDEX_INVALID);
}

inline void
appendPlanControlVertices(
    Neighborhood const & neighborhood, std::vector<Index> & controls) {

    // XXXX manuelk need to find a better way...
    ConstIndexArray cvs = neighborhood.GetVertRemaps();
    for (int i=0; i<cvs.size(); ++i) {
        controls.push_back(cvs[i]);
    }
}

SubdivisionPlanTable const *
CharacteristicMap::HashTopology(TopologyRefiner const & refiner) {

    if (!supportsEndCaps(_options.GetEndCapType()) || _options.hashSize<=0) {
        return 0;
    }

    internal::CharacteristicBuilder charBuilder(refiner, *this);

    TopologyLevel const & coarseLevel = refiner.GetLevel(0);

    int regFaceSize = Sdc::SchemeTypeTraits::GetRegularFaceSize(refiner.GetSchemeType());

    int nfaces = coarseLevel.GetNumFaces(),
        maxValence = refiner.GetMaxValence(),
        nplans = SubdivisionPlanTable::countNumPlans(coarseLevel, regFaceSize),
        regValence = Sdc::SchemeTypeTraits::GetRegularFaceSize(refiner.GetSchemeType()),
        hashSize = _options.hashSize;

    SubdivisionPlanTable * plansTable = new SubdivisionPlanTable(*this);

    SubdivisionPlanVector & plans = plansTable->_plans;
    plans.reserve(nplans);

    std::vector<Index> & controls = plansTable->_controlVertices;
    controls.reserve(nplans*20);

    // hash topology : faces with redundant topological configurations share
    // the same characteristic
    NeighborhoodBuilder neighborhoodBuilder(maxValence);

    std::vector<Neighborhood const *> neighborhoods;
    neighborhoods.reserve(nplans);

    for (int faceIndex=0, firstControl=0; faceIndex < nfaces; ++faceIndex) {

        if (coarseLevel.IsFaceHole(faceIndex)) {
            plans.push_back(SubdivisionPlan(0, INDEX_INVALID, INDEX_INVALID));
            continue;
        }

        Neighborhood const * neighborhood =
            neighborhoodBuilder.Create(coarseLevel, faceIndex);

        int rotation=0;

        Index charIndex = findCharacteristic(*neighborhood, &rotation);

        if (charIndex==INDEX_INVALID) {

            // this topological configuration does not exist in the map :
            // create a new characteristic & add it to the map
            charIndex = GetNumCharacteristics();
            assert(charIndex!=INDEX_INVALID);

            int valence = neighborhood->GetValence();
            if (valence!=regValence) {
                ConstIndexArray childFaces =
                    coarseLevel.GetFaceChildFaces(faceIndex);
                for (int i=0; i<valence; ++i) {
                    Characteristic const * ch =
                        charBuilder.Create(1, childFaces[i], neighborhood);
                    _characteristics.push_back(ch);
                }
            } else {
                Characteristic const * ch =
                    charBuilder.Create(0, faceIndex, neighborhood);
                _characteristics.push_back(ch);
            }

            addCharacteristicToHash(
                coarseLevel, neighborhoodBuilder, faceIndex, charIndex, valence);
        } else {
            if (rotation) {
                free((void *)neighborhood);
                neighborhood =
                    neighborhoodBuilder.Create(coarseLevel, faceIndex, rotation);
            }
            neighborhoods.push_back(neighborhood);
        }

        int numControlVertices =
            GetCharacteristic(charIndex)->GetNumControlVertices();

        ConstIndexArray fverts = coarseLevel.GetFaceVertices(faceIndex);
        if (fverts.size()==regFaceSize) {

            plans.push_back(
                SubdivisionPlan(numControlVertices, firstControl, charIndex));

            appendPlanControlVertices(*neighborhood, controls);

            firstControl += numControlVertices;
        } else {

            for (int i=0; i<fverts.size(); ++i) {

                plans.push_back(
                    SubdivisionPlan(numControlVertices, firstControl, charIndex+i));

                appendPlanControlVertices(*neighborhood, controls);
                firstControl += numControlVertices;
            }
        }
    }

    _numMaxSupports = std::max(
        _numMaxSupports, charBuilder.FinalizeSupportStencils());

    for (int i=0; i<(int)neighborhoods.size(); ++i) {
        free((void *)neighborhoods[i]);
    }

    return plansTable;
}

int
CharacteristicMap::GetCharacteristicTreeSizeTotal() const {

    int size = 0;
    for (int i=0; i<GetNumCharacteristics(); ++i) {
        Characteristic const * ch = GetCharacteristic(i);
        size += ch->GetTreeSize();
    }
    return size;
}

void
CharacteristicMap::WriteCharacteristicsDigraphs(
    FILE * fout, char const * title, bool showIndices) const {

    fprintf(fout, "digraph CharacteristicMap {\n");

    if (title) {
        fprintf(fout, "  label = \"%s\";\n", title);
        fprintf(fout, "  labelloc = \"t\";\n");
    }

    for (int charIndex=0; charIndex<GetNumCharacteristics(); ++charIndex) {

        Characteristic const * ch = GetCharacteristic(charIndex);
        if (ch) {
            ch->WriteTreeDigraph(fout, charIndex, showIndices, /*isSubgraph*/ true);
        }
    }
    fprintf(fout, "}\n");
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
