//
//   Copyright 2013 Nvidia
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

#include <far/characteristicFactory.h>
#include <far/patchFaceTag.h>
#include <far/topologyRefinerFactory.h>

#include "init_shapes.h"

#include <string>
#include <fstream>
#include <sstream>

using namespace OpenSubdiv;

static void
testMe(ShapeDesc const & shapeDesc, int maxlevel=3) {

    Shape const * shape = Shape::parseObj(
        shapeDesc.data.c_str(), shapeDesc.scheme);

    // create Far mesh (topology)
    Sdc::SchemeType sdctype = GetSdcType(*shape);
    Sdc::Options    sdcoptions = GetSdcOptions(*shape);

    Far::TopologyRefiner * refiner =
        Far::TopologyRefinerFactory<Shape>::Create(*shape,
            Far::TopologyRefinerFactory<Shape>::Options(sdctype, sdcoptions));

    delete shape;

    // refine adaptively
    {
        Far::TopologyRefiner::AdaptiveOptions options(maxlevel);
        refiner->RefineAdaptive(options);
    }

    // identify patch types
    Far::PatchFaceTagVector patchTags;
    Far::PatchFaceTag::IdentifyAdaptivePatches(
        *refiner, patchTags, maxlevel, false /*single crease*/);

    // build characteristics
    Far::CharacteristicFactory::Options options;
    Far::Characteristic const * ch =
        Far::CharacteristicFactory::Create(*refiner, patchTags, options);

    delete [] ch;
}



int main(int argc, char **argv) {

    std::string str;

    if (argc > 1) {
        std::ifstream ifs(argv[1]);
        if (ifs) {
            std::stringstream ss;
            ss << ifs.rdbuf();
            ifs.close();
            str = ss.str();
            g_defaultShapes.push_back(ShapeDesc(argv[1], str.c_str(), kCatmark));
        }
    }

    initShapes();

    ShapeDesc const & sdesc = g_defaultShapes[8]; // g_defaultShapes[42];
    int level=3;
//printf("Shape='%s' (lvl=%d)\n", sdesc.name.c_str(), level);
    testMe(sdesc, level);
}
