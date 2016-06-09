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

#include <far/characteristicMapFactory.h>
#include <far/patchFaceTag.h>
#include <far/stencilTable.h>
#include <far/topologyRefinerFactory.h>

#include "init_shapes.h"

#include <string>
#include <fstream>
#include <sstream>

using namespace OpenSubdiv;

struct Vertex {

    // Minimal required interface ----------------------
    Vertex() { }

    void Clear( void * =0 ) {
         point[0] = point[1] = point[2] = 0.0f;
    }

    void AddWithWeight(Vertex const & src, float weight) {
        point[0] += weight * src.point[0];
        point[1] += weight * src.point[1];
        point[2] += weight * src.point[2];
    }

    float point[3];
};

struct LimitFrame {

    void Clear( void * =0 ) {
         point[0] =  point[1] =  point[2] = 0.0f;
        deriv1[0] = deriv1[1] = deriv1[2] = 0.0f;
        deriv2[0] = deriv2[1] = deriv2[2] = 0.0f;
    }

    void AddWithWeight(Vertex const & src,
        float weight, float d1Weight, float d2Weight) {

        point[0] += weight * src.point[0];
        point[1] += weight * src.point[1];
        point[2] += weight * src.point[2];

        deriv1[0] += d1Weight * src.point[0];
        deriv1[1] += d1Weight * src.point[1];
        deriv1[2] += d1Weight * src.point[2];

        deriv2[0] += d2Weight * src.point[0];
        deriv2[1] += d2Weight * src.point[1];
        deriv2[2] += d2Weight * src.point[2];
    }

    float point[3],
          deriv1[3],
          deriv2[3];
};

//------------------------------------------------------------------------------

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

    // refine adaptively
    {
        Far::TopologyRefiner::AdaptiveOptions options(maxlevel);
        refiner->RefineAdaptive(options);
    }

    // identify patch types
    Far::PatchFaceTagVector patchTags;
    Far::PatchFaceTag::IdentifyAdaptivePatches(
        *refiner, patchTags, maxlevel, false /*single crease*/);

    // build characteristics map
    Far::CharacteristicMapFactory::Options options;
    Far::CharacteristicMap const * charmap =
        Far::CharacteristicMapFactory::Create(*refiner, patchTags, options);
/*
    // create vertex primvar data buffer
    std::vector<Vertex> vertexBuffer;
    {
        int numVertsTotal = refiner->GetNumVerticesTotal();
        if (charmap->GetLocalPointStencilTable()) {
            numVertsTotal += charmap->GetLocalPointStencilTable()->GetNumStencils();
        }

        vertexBuffer.resize(numVertsTotal);
        Vertex * verts = &vertexBuffer[0];

        // copy coarse vertices positions
        int ncoarseverts = shape->GetNumVertices();
        for (int i=0; i<ncoarseverts; ++i) {
            float const * ptr = &shape->verts[i*3];
            verts[i].SetPosition(ptr[0], ptr[1], ptr[2]);
        }

        // populate primvar buffer with Far interpolated vertex data
        {
            // TopologyRefiner interpolation
            Vertex * src = verts;
            for (int level = 1; level <= refiner->GetMaxLevel(); ++level) {
                Vertex * dst = src + refiner->GetLevel(level-1).GetNumVertices();
                Far::PrimvarRefiner(*refiner).Interpolate(level, src, dst);
                src = dst;
            }

            // endpatch basis conversion
            Far::StencilTable const * localPoints =
                charmap->GetLocalPointStencilTable();

            localPoints->UpdateValues(verts, verts + refiner->GetNumVerticesTotal());
        }
    }

    delete shape;

    // tessellate
    for (int i=0; i<charmap->GetNumCharacteristics(); ++i) {

        Far::Characteristic const & ch = charmap->GetCharacteristic(i);

        float wP[20], wDs[20], wDt[20];

        int tessFactor = 5;
        for (int y=0; y<tessFactor; ++y) {
            for (int x=0; x<tessFactor; ++x) {

                float s = (float)x / (float)tessFactor,
                      t = (float)y / (float)tessFactor;

                ch.EvaluateBasis(s, t, wP, wDs, wDt);
            }
        }
    }
*/
    delete charmap;
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
