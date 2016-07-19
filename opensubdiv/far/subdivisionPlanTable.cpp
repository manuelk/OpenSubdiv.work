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

#include "../far/subdivisionPlanTable.h"
#include "../far/characteristicMap.h"
#include "../far/characteristicBuilder.h"
#include "../far/topologyRefiner.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

SubdivisionPlanTable::SubdivisionPlanTable(
    CharacteristicMap const * charmap) : _charmap(charmap) {
}

SubdivisionPlanTable::~SubdivisionPlanTable() {
    // If the characteristics map is set to hashing, then client code owns it.
    // Otherwise, the SubdivisionPlanTable allocated it and is responsible for
    // destroying it.
    if (_charmap->GetOptions().hashSize==0) {
        delete _charmap;
    }
}

int
SubdivisionPlanTable::countPlans(
    TopologyLevel const & coarseLevel, int regFaceSize) {

    int nfaces = coarseLevel.GetNumFaces(),
        nplans = 0;

    // non-quads require 1 plan for each face-vert
    for (int face = 0; face < nfaces; ++face) {
        if (coarseLevel.IsFaceHole(face)) {
            continue;
        }
        ConstIndexArray fverts = coarseLevel.GetFaceVertices(face);
        nplans += fverts.size()==regFaceSize ? 1 : fverts.size();
    }
    return nplans;
}

SubdivisionPlanTable const *
SubdivisionPlanTable::Create(TopologyRefiner const & refiner, Options options) {

    if (!CharacteristicMap::supportsEndCaps(options.GetEndCapType())) {
        return 0;
    }

    TopologyLevel const & coarseLevel = refiner.GetLevel(0);

    int regFaceSize = Sdc::SchemeTypeTraits::GetRegularFaceSize(refiner.GetSchemeType());

    int nfaces = coarseLevel.GetNumFaces(),
        nplans = countPlans(coarseLevel, regFaceSize);

    // note : this map is not set to hash topology and we own it (destructor
    //        will delete it)
    options.hashSize = 0;
    CharacteristicMap * charmap = new CharacteristicMap(options);
    charmap->_characteristics.reserve(nplans);

    CharacteristicBuilder charBuilder(refiner, *charmap);

    SubdivisionPlanTable * table = new SubdivisionPlanTable(charmap);
    table->_plans.reserve(nplans);

    int rootNodeOffset = 0;
    for (int face = 0; face < nfaces; ++face) {

        if (coarseLevel.IsFaceHole(face)) {
            continue;
        }

        ConstIndexArray verts = coarseLevel.GetFaceVertices(face);
        if (verts.size()==regFaceSize) {

            Characteristic * ch = new Characteristic(charmap);
            ch->writeCharacteristicTree(charBuilder, 0, face);
            charmap->_characteristics.push_back(ch);

            SubdivisionPlan plan;
            plan.charIndex = (int)charmap->_characteristics.size()-1;
            plan.rootNodeOffset = rootNodeOffset;
            table->_plans.push_back(plan);
            rootNodeOffset += ch->GetTreeSize();
        } else {
            ConstIndexArray children = coarseLevel.GetFaceChildFaces(face);
            for (int i=0; i<children.size(); ++i) {

                Characteristic * ch = new Characteristic(charmap);
                ch->writeCharacteristicTree(charBuilder, 1, children[i]);
                charmap->_characteristics.push_back(ch);

                SubdivisionPlan plan;
                plan.charIndex = (int)charmap->_characteristics.size()-1;
                plan.rootNodeOffset = rootNodeOffset;
                table->_plans.push_back(plan);
                rootNodeOffset += ch->GetTreeSize();
            }
        }
    }
    charmap->_localPointStencils = charBuilder.FinalizeStencils();
    charmap->_localPointVaryingStencils = charBuilder.FinalizeVaryingStencils();
    return table;
}



} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv

