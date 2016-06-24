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

#if defined(__APPLE__)
    #if defined(OSD_USES_GLEW)
        #include <GL/glew.h>
    #else
        #include <OpenGL/gl3.h>
    #endif
    #define GLFW_INCLUDE_GL3
    #define GLFW_NO_GLU
#else
    #include <stdlib.h>
    #include <GL/glew.h>
    #if defined(WIN32)
        #include <GL/wglew.h>
    #endif
#endif

#include <GLFW/glfw3.h>
GLFWwindow* g_window=0;
GLFWmonitor* g_primary=0;

#include <osd/glVertexBuffer.h>

#include <far/characteristicMapFactory.h>
#include <far/patchFaceTag.h>
#include <far/patchTableFactory.h>
#include <far/stencilTable.h>
#include <far/topologyRefinerFactory.h>

#include "../common/stopwatch.h"
#include "../common/simple_math.h"
#include "../common/glUtils.h"
#include "../common/glControlMeshDisplay.h"
#include "../common/glHud.h"

#include "./init_shapes.h"
#include "./glFontutils.h"
#include "./glMesh.h"

#include <string>
#include <fstream>
#include <sstream>

using namespace OpenSubdiv;

enum DisplayStyle { DISPLAY_STYLE_WIRE,
                    DISPLAY_STYLE_SHADED,
                    DISPLAY_STYLE_WIRE_ON_SHADED };

enum ShadingMode {
    SHADING_PATCH_TYPE,
    SHADING_PATCH_COORD,
    SHADING_PATCH_NORMAL,
};

int g_level = 2,
    g_shadingMode = SHADING_PATCH_TYPE,
    g_tessLevel = 10,
    g_tessLevelMin = 2,
    g_currentShape = 8, //cube = 8 square = 12 pyramid = 45 torus = 49
    g_useTerminalNodes = true;

int   g_frame = 0,
      g_repeatCount = 0;

bool g_DrawNodeIDs = false;

Far::EndCapType g_endCap = Far::ENDCAP_BSPLINE_BASIS;

// GUI variables
int   g_fullscreen = 0,
      g_mbutton[3] = {0, 0, 0},
      g_running = 1,
      g_displayStyle = DISPLAY_STYLE_WIRE;

float g_rotate[2] = {0, 0},
      g_dolly = 5,
      g_pan[2] = {0, 0},
      g_center[3] = {0, 0, 0},
      g_size = 0;

int   g_prev_x = 0,
      g_prev_y = 0;

int   g_width = 1024,
      g_height = 1024;

GLhud g_hud;

Stopwatch g_fpsTimer;

GLuint g_transformUB = 0,
       g_lightingUB = 0;

struct Transform {
    float ModelViewMatrix[16];
    float ProjectionMatrix[16];
    float ModelViewProjectionMatrix[16];
} g_transformData;

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

static float g_patchColors[43][4] = {
        {1.0f,  1.0f,  1.0f,  1.0f},   // regular
        {0.0f,  1.0f,  1.0f,  1.0f},   // regular pattern 0
        {0.0f,  0.5f,  1.0f,  1.0f},   // regular pattern 1
        {0.0f,  0.5f,  0.5f,  1.0f},   // regular pattern 2
        {0.5f,  0.0f,  1.0f,  1.0f},   // regular pattern 3
        {1.0f,  0.5f,  1.0f,  1.0f},   // regular pattern 4

        {1.0f,  0.5f,   0.5f,  1.0f},  // single crease
        {1.0f,  0.70f,  0.6f,  1.0f},  // single crease pattern 0
        {1.0f,  0.65f,  0.6f,  1.0f},  // single crease pattern 1
        {1.0f,  0.60f,  0.6f,  1.0f},  // single crease pattern 2
        {1.0f,  0.55f,  0.6f,  1.0f},  // single crease pattern 3
        {1.0f,  0.50f,  0.6f,  1.0f},  // single crease pattern 4

        {0.8f,  0.0f,  0.0f,  1.0f},   // boundary
        {0.0f,  0.0f,  0.75f, 1.0f},   // boundary pattern 0
        {0.0f,  0.2f,  0.75f, 1.0f},   // boundary pattern 1
        {0.0f,  0.4f,  0.75f, 1.0f},   // boundary pattern 2
        {0.0f,  0.6f,  0.75f, 1.0f},   // boundary pattern 3
        {0.0f,  0.8f,  0.75f, 1.0f},   // boundary pattern 4

        {0.0f,  1.0f,  0.0f,  1.0f},   // corner
        {0.25f, 0.25f, 0.25f, 1.0f},   // corner pattern 0
        {0.25f, 0.25f, 0.25f, 1.0f},   // corner pattern 1
        {0.25f, 0.25f, 0.25f, 1.0f},   // corner pattern 2
        {0.25f, 0.25f, 0.25f, 1.0f},   // corner pattern 3
        {0.25f, 0.25f, 0.25f, 1.0f},   // corner pattern 4

        {1.0f,  1.0f,  0.0f,  1.0f},   // gregory
        {1.0f,  1.0f,  0.0f,  1.0f},   // gregory
        {1.0f,  1.0f,  0.0f,  1.0f},   // gregory
        {1.0f,  1.0f,  0.0f,  1.0f},   // gregory
        {1.0f,  1.0f,  0.0f,  1.0f},   // gregory
        {1.0f,  1.0f,  0.0f,  1.0f},   // gregory

        {1.0f,  0.5f,  0.0f,  1.0f},   // gregory boundary
        {1.0f,  0.5f,  0.0f,  1.0f},   // gregory boundary
        {1.0f,  0.5f,  0.0f,  1.0f},   // gregory boundary
        {1.0f,  0.5f,  0.0f,  1.0f},   // gregory boundary
        {1.0f,  0.5f,  0.0f,  1.0f},   // gregory boundary
        {1.0f,  0.5f,  0.0f,  1.0f},   // gregory boundary

        {1.0f,  0.7f,  0.3f,  1.0f},   // gregory basis
        {1.0f,  0.7f,  0.3f,  1.0f},   // gregory basis
        {1.0f,  0.7f,  0.3f,  1.0f},   // gregory basis
        {1.0f,  0.7f,  0.3f,  1.0f},   // gregory basis
        {1.0f,  0.7f,  0.3f,  1.0f},   // gregory basis
        {1.0f,  0.7f,  0.3f,  1.0f},   // gregory basis

        {0.4f,  0.4f,  0.6f,  1.0f},   // terminal
};

static float const *
getAdaptiveColor(Far::Characteristic::Node node) {

    Far::Characteristic::NodeDescriptor desc = node.GetDescriptor();

    int patchType = 0;
    if (desc.GetType()==Far::Characteristic::NODE_REGULAR) {

        int edgeCount =desc.GetBoundaryCount();
        switch (edgeCount) {
            case 1 : patchType = 2; break;
            case 2 : patchType = 3; break;
        };

        if (desc.SingleCrease()) {
            patchType = 1;
        }
    } else if (desc.GetType()==Far::Characteristic::NODE_END) {

        Far::EndCapType type =
            node.GetCharacteristic()->GetCharacteristicMap()->GetEndCapType();
        switch (type) {
            case Far::ENDCAP_BILINEAR_BASIS : break;
            case Far::ENDCAP_BSPLINE_BASIS : break;
            case Far::ENDCAP_GREGORY_BASIS : patchType = 6; break;
            default:
                break;
        }
    } else if (desc.GetType()==Far::Characteristic::NODE_TERMINAL) {
        patchType = 7;
    } else {
        assert(0);
    }

    int pattern = desc.GetTransitionCount();

    return g_patchColors[6*patchType + pattern];
}

//------------------------------------------------------------------------------

static void
printIndices(Far::Index const * cvs, int ncvs) {
    int stride = ncvs==16 ? 4 : 5;
    for (int i=0; i<ncvs; ++i) {
        if (i>0 && ((i%stride)!=0))
            printf(" ");
        if ((i%stride)==0)
            printf("\n");
        printf("%*d", 4, cvs[i]);
    }
}

static void
printBasis(float const * basis) {
    for (int j=0; j<4; ++j) {
        for (int i=0; i<4; ++i) {
            printf("%f, ", basis[4*j+i]);
        }
        printf("\n");
    }
}


static void
printArray(Far::ConstIndexArray array) {
    printf("[");
    for (int i=0; i<array.size(); ++i) {
        if (i>0) printf(", ");
        printf("%3d", array[i]);
    }
    printf("]");
}

static void
printPatchTables(Far::TopologyRefiner const * refiner) {

    Far::PatchTableFactory::Options options;
    options.SetEndCapType(g_endCap);
    Far::PatchTable const * patchTable =
        Far::PatchTableFactory::Create(*refiner, options);
    printf("PatchTables :\n");

    int narrays = patchTable->GetNumPatchArrays(),
        patchCount = 0;
    for (int array=0; array<narrays; ++array) {
        int npatches = patchTable->GetNumPatches(array);
        for (int patch=0; patch<npatches; ++patch, ++patchCount) {

            printf("  ID = %3d ", patchCount);
            patchTable->GetPatchParam(array, patch).Print();
            printf(" ");

            Far::ConstIndexArray cvs =
                patchTable->GetPatchVertices(array, patch);
            printArray(cvs);
            printf("\n");
        }
    }
    printf("\n");
    fflush(stdout);
    delete patchTable;
}

static void
printCharmapNodes(Far::CharacteristicMap const * charmap) {

    typedef Far::Characteristic::Node Node;
    typedef Far::Characteristic::NodeDescriptor NodeDesc;

    printf("Nodes :\n");

    int nchars = charmap->GetNumCharacteristics();
    for (int i=0; i<nchars; ++i) {

        Far::Characteristic const & ch = charmap->GetCharacteristic(i);

        printf("Characteristic : %d\n", i);

        int nodeIndex = 0;
        for (Node it = ch.GetTreeNode(0); it.GetTreeOffset()<ch.GetTreeSize(); ++it, ++nodeIndex) {

            NodeDesc desc = it.GetDescriptor();

            if (desc.GetType()!=Far::Characteristic::NODE_REGULAR &&
                desc.GetType()!=Far::Characteristic::NODE_END) {
                continue;
            }

            printf("    ");

            Far::PatchParam param;
            param.Set(/*face id*/ 0, desc.GetU(), desc.GetV(), desc.GetDepth(),
                desc.NonQuadRoot(), desc.GetBoundaryMask(), desc.GetTransitionMask());
            param.Print();
            printf(" ");

            Far::ConstIndexArray supportIndices = it.GetSupportIndices();
            printArray(supportIndices);
            printf(" ID=%3d type=%d offset=%4d screase=%d", nodeIndex, desc.GetType(), it.GetTreeOffset(), desc.SingleCrease());
            float const * c = getAdaptiveColor(it);
            printf(" bcount=%d color=(%f %f %f)", desc.GetBoundaryCount(), c[0], c[1], c[2]);
            printf("\n");
        }
    }
    fflush(stdout);
}

//------------------------------------------------------------------------------

GLFont * g_font=0;

static void
createNodeIDs(Far::CharacteristicMap const & charmap,
    std::vector<Vertex> const & vertexBuffer) {

    typedef Far::Characteristic::Node Node;
    typedef Far::Characteristic::NodeDescriptor NodeDesc;

    int nchars = charmap.GetNumCharacteristics();

    for (int i=0; i<nchars; ++i) {

        Far::Characteristic const & ch = charmap.GetCharacteristic(i);

        Node it = ch.GetTreeNode(0);
        for (int nindex=0; it.GetTreeOffset()<ch.GetTreeSize(); ++nindex, ++it) {

            NodeDesc desc = it.GetDescriptor();

            if (desc.GetType()==Far::Characteristic::NODE_RECURSIVE)
                continue;

            OpenSubdiv::Far::ConstIndexArray const cvs = it.GetSupportIndices();

            Vertex center;
            center.Clear();

            float weight = 1.0f / cvs.size();
            for (int k=0; k<cvs.size(); ++k) {
                int support = cvs[k];
                if (support!=Far::INDEX_INVALID) {
                    center.AddWithWeight(vertexBuffer[support], weight);
                }
            }

            static char buf[16];
            snprintf(buf, 16, "%d-%d", i, nindex);
            g_font->Print3D(center.point, buf, 1);
        }
    }

}

//------------------------------------------------------------------------------
int g_currentCharIndex = 0,
    g_currentNodeIndex = -1;

Far::CharacteristicMap const * g_charmap = 0;

Far::Characteristic::Node g_currentNode;

static int remapTerminalIndices[4][16] = {
    {0, 1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18},
    {1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19},
    {5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18, 20, 21, 22, 23}, // note : winding order is 0, 1, 3, 2
    {6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24},
};

static void
createNodeNumbers(Far::CharacteristicMap const * charmap,
    std::vector<Vertex> const & vertexBuffer) {

    if (!charmap) {
        return;
    }

    g_currentCharIndex = std::max(0,
        std::min(g_currentCharIndex, charmap->GetNumCharacteristics()-1));

    g_currentNodeIndex = std::max(-1, g_currentNodeIndex);

    if (g_currentCharIndex>=0 && g_currentNodeIndex>=0) {

        Far::Characteristic const & ch =
            charmap->GetCharacteristic(g_currentCharIndex);

        Far::Characteristic::Node node = ch.GetTreeNode(0);

        for (int count=0; node.GetTreeOffset()<ch.GetTreeSize(); ++node, ++count) {

            if (count==g_currentNodeIndex) {

                g_currentNode = node;

                Far::ConstIndexArray supports = node.GetSupportIndices();
#if 0
                Far::Characteristic::NodeDescriptor desc = node.GetDescriptor();

                if (desc.GetType()==Far::Characteristic::NODE_TERMINAL) {
                    Far::Index quadSupports[16];

                    for (int quadrant=0; quadrant<4; ++quadrant) {
                         printf("\nchar=%d node=%d quadrant=%d ev=%d",
                             g_currentCharIndex, g_currentNodeIndex, quadrant, desc.GetEvIndex());
                         for (int k=0; k<16; ++k) {
                             quadSupports[k] = supports[remapTerminalIndices[quadrant][k]];
                         }
                         printIndices(quadSupports, 16);
                    }
                    printf("\n------------\n");
                    fflush(stdout);
                }
#endif
                for (int k=0; k<supports.size(); ++k) {

                    Far::Index vertIndex = supports[k];

                    if (vertIndex!=Far::INDEX_INVALID) {
                        Vertex const & vert = vertexBuffer[vertIndex];

                        float position[3] = { vert.point[0], vert.point[1], vert.point[2] + k*0.001f, };

                        static char buf[16];
                        snprintf(buf, 16, "%d", k);
                        g_font->Print3D(position, buf, 2);
                    }
                }
                return;
            }
        }
    }
    g_currentNodeIndex = -1;
}

//------------------------------------------------------------------------------

GLMesh * g_tessMesh = 0;

static void
createTessMesh(Far::CharacteristicMap const * charmap, int tessFactor,
    std::vector<float> positions, std::vector<float> normals, std::vector<float> colors,
        bool createPointsMesh=false) {

    delete g_tessMesh;
    if (createPointsMesh) {
        struct GLMesh::Topology topo;
        topo.positions = &positions[0];
        topo.normals = &normals[0];
        topo.colors = &colors[0];
        topo.nverts = (int)positions.size()/3;
        g_tessMesh = new GLMesh(topo, GLMesh::DRAW_POINTS);
    } else {
        int nchars = charmap->GetNumCharacteristics(),
            ntriangles = 2 * (tessFactor-1) * (tessFactor-1) * nchars;

        // create tess mesh

        struct GLMesh::Topology topo;
        topo.positions = new float[ntriangles * 3 * 3];
        topo.normals = new float[ntriangles * 3 * 3];
        topo.colors = new float[ntriangles * 3 * 3];
        topo.nverts = ntriangles * 3;

        float * pos = topo.positions,
              * norm = topo.normals,
              * col = topo.colors;

        for (int i=0, charOffset=0; i<nchars; ++i) {

            Far::Characteristic const & ch = charmap->GetCharacteristic(i);

            if (ch.GetTreeSize()==0) {
                continue;
            }

            // generate indices
            for (int y=0; y<(tessFactor-1); ++y) {
                for (int x=0; x<(tessFactor-1); ++x) {
                                                                         //  A o----o B
                    int A = 3 * (charOffset + y * tessFactor + x),       //    |\   |
                        B = 3 * (charOffset + y * tessFactor + x+1),     //    | \  |
                        C = 3 * (charOffset + (y+1) * tessFactor + x),   //    |  \ |
                        D = 3 * (charOffset + (y+1) * tessFactor + x+1); //    |   \|
                                                                         //  C o----o D
                    assert(A < positions.size() && B < positions.size() &&
                        C < positions.size() && D < positions.size());

                    // first triangle
                    memcpy(pos, &positions[A],  3 * sizeof(float)); pos+=3;
                    memcpy(norm, &normals[A], 3 * sizeof(float)); norm+=3;
                    memcpy(col, &colors[A],  3 * sizeof(float)); col+=3;

                    memcpy(pos, &positions[D],  3 * sizeof(float)); pos+=3;
                    memcpy(norm, &normals[D], 3 * sizeof(float)); norm+=3;
                    memcpy(col, &colors[D],  3 * sizeof(float)); col+=3;

                    memcpy(pos, &positions[C],  3 * sizeof(float)); pos+=3;
                    memcpy(norm, &normals[C], 3 * sizeof(float)); norm+=3;
                    memcpy(col, &colors[C],  3 * sizeof(float)); col+=3;

                    // second triangle
                    memcpy(pos, &positions[A],  3 * sizeof(float)); pos+=3;
                    memcpy(norm, &normals[A], 3 * sizeof(float)); norm+=3;
                    memcpy(col, &colors[A],  3 * sizeof(float)); col+=3;

                    memcpy(pos, &positions[B],  3 * sizeof(float)); pos+=3;
                    memcpy(norm, &normals[B], 3 * sizeof(float)); norm+=3;
                    memcpy(col, &colors[B],  3 * sizeof(float)); col+=3;

                    memcpy(pos, &positions[D],  3 * sizeof(float)); pos+=3;
                    memcpy(norm, &normals[D], 3 * sizeof(float)); norm+=3;
                    memcpy(col, &colors[D],  3 * sizeof(float)); col+=3;
                }
            }
            charOffset += tessFactor * tessFactor;
        }
        g_tessMesh = new GLMesh(topo);

        delete [] topo.positions;
        delete [] topo.normals;
        delete [] topo.colors;
    }
}

//------------------------------------------------------------------------------

#define CREATE_SHAPE_TESS
#ifdef CREATE_SHAPE_TESS

GLControlMeshDisplay g_controlMeshDisplay;

Osd::GLVertexBuffer * g_controlMeshVerts = 0;

static void
createMesh(ShapeDesc const & shapeDesc, int maxlevel=3) {

    Shape const * shape = Shape::parseObj(
        shapeDesc.data.c_str(), shapeDesc.scheme);

    // create Far mesh (topology)
    Sdc::SchemeType sdctype = GetSdcType(*shape);
    Sdc::Options    sdcoptions = GetSdcOptions(*shape);

    Far::TopologyRefiner * refiner =
        Far::TopologyRefinerFactory<Shape>::Create(*shape,
            Far::TopologyRefinerFactory<Shape>::Options(sdctype, sdcoptions));

    // control mesh
    delete g_controlMeshVerts;
    int nverts = shape->GetNumVertices();
    g_controlMeshVerts = Osd::GLVertexBuffer::Create(3, nverts);
    g_controlMeshVerts->UpdateData(&shape->verts[0], 0, nverts);
    g_controlMeshDisplay.SetTopology(refiner->GetLevel(0));

    bool useSingleCreasePatches = true;

    // refine adaptively
    {
        Far::TopologyRefiner::AdaptiveOptions options(maxlevel);
        options.useSingleCreasePatch = useSingleCreasePatches;
        refiner->RefineAdaptive(options);
    }

    // identify patch types
    Far::PatchFaceTagVector patchTags;
    Far::PatchFaceTag::IdentifyAdaptivePatches(
        *refiner, patchTags, maxlevel, useSingleCreasePatches);

    // build characteristics map

    delete g_charmap;
    Far::CharacteristicMapFactory::Options options;
    options.endCapType = g_endCap;
    options.useTerminalNodes = g_useTerminalNodes;
    Far::CharacteristicMap const * charmap =
        Far::CharacteristicMapFactory::Create(*refiner, patchTags, options);

    // create vertex primvar data buffer
    std::vector<Vertex> supportsBuffer;
    {
        int numVertsTotal = refiner->GetNumVerticesTotal();
        if (charmap->GetLocalPointStencilTable()) {
            numVertsTotal += charmap->GetLocalPointStencilTable()->GetNumStencils();
        }

        supportsBuffer.resize(numVertsTotal);
        Vertex * verts = &supportsBuffer[0];

        // copy coarse vertices positions
        int ncoarseverts = shape->GetNumVertices();
        for (int i=0; i<ncoarseverts; ++i) {
            float const * ptr = &shape->verts[i*3];
            memcpy(verts[i].point, ptr, 3 * sizeof(float));
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
            if (localPoints) {
                localPoints->UpdateValues(verts, verts + refiner->GetNumVerticesTotal());
            }
        }
    }

    delete shape;

    g_charmap = charmap;

    // draw selected node data
    createNodeNumbers(g_charmap, supportsBuffer);


    //
    // tessellate
    //

    int const tessFactor = g_tessLevel;

//#define DEBUG_PRINT
#ifdef DEBUG_PRINT
    printPatchTables(refiner);
    printCharmapNodes(charmap);
#endif

    if (g_DrawNodeIDs) {
        createNodeIDs(*charmap, supportsBuffer);
    }

    int nchars = charmap->GetNumCharacteristics();


    // interpolate limits

#define TESS_DOMAIN
#ifdef TESS_DOMAIN
    nverts = tessFactor * tessFactor * nchars;

    std::vector<float> positions(3 * nverts),
                       normals(3 * nverts),
                       colors(3 * nverts);

    float * pos = &positions[0],
          * norm = &normals[0],
          * col = &colors[0];

    for (int i=0; i<nchars; ++i) {

        Far::Characteristic const & ch = charmap->GetCharacteristic(i);

        if (ch.GetTreeSize()==0) {
            continue; // skip holes
        }

        // interpolate vertices
        float wP[20], wDs[20], wDt[20];

        for (int y=0; y<tessFactor; ++y) {
            for (int x=0; x<tessFactor; ++x, pos+=3, norm+=3, col+=3) {

                // compute basis weights at location (s,t)
                float s = (float)x / (float)(tessFactor-1),
                      t = (float)y / (float)(tessFactor-1);

                unsigned char quadrant=0;

                Far::Characteristic::Node node =
                    ch.EvaluateBasis(s, t, wP, wDs, wDt, &quadrant);

                Far::Characteristic::NodeDescriptor desc = node.GetDescriptor();

                Far::ConstIndexArray supportIndices = node.GetSupportIndices();

                Far::Index supports[16];
                if (desc.GetType()==Far::Characteristic::NODE_TERMINAL) {
                    node.GetSupportIndices(quadrant, supports);
                    supportIndices = Far::ConstIndexArray(supports, 16);
                }
//printf("s=%f t=%f depth=%d quadrant=%d\n", s, t, node.GetDescriptor().GetDepth(), quadrant);
//printBasis(wP);
//printf("supports=");
//printIndices(supportIndices.begin(), 16);
//printf("\n------------------------\n");

                // interpolate support points with basis weights
                LimitFrame limit;
                limit.Clear();
                for (int k=0; k<supportIndices.size(); ++k) {
                    assert(supportIndices[k]<(int)supportsBuffer.size());

                    if (supportIndices[k]==-1)
                        continue;

                    Vertex const & support = supportsBuffer[supportIndices[k]];
                    limit.AddWithWeight(support, wP[k], wDs[k], wDt[k]);
                }

                memcpy(pos, limit.point, 3 * sizeof(float));

                //float n[3] = { 0.0f, 0.0f, 0.0f };
                cross(norm, limit.deriv1, limit.deriv2 );

                //float const * c = g_palette[ch.GetNodeIndex(node) % 7];

                bool nodeSelected = false;
                if (g_charmap && g_currentCharIndex>=0 &&
                    g_currentNodeIndex>=0 && g_currentNode==node) {
                    nodeSelected = true;
                }

                if (nodeSelected) {
                    static float selColor[3] = { 0.0f, 255.0f, 0.0f },
                                 quadColors[4][3] = {{ 255.0f,    0.0f,   0.0f },
                                                     {   0.0f,  255.0f,   0.0f },
                                                     {   0.0f,    0.0f, 255.0f },
                                                     {  255.0f, 255.0f,   0.0f }};

                    if (desc.GetType()==Far::Characteristic::NODE_TERMINAL) {
                        memcpy(col, quadColors[quadrant], 3 * sizeof(float));
                    } else {
                        memcpy(col, selColor, 3 * sizeof(float));
                    }
                } else {
                    switch (g_shadingMode) {
                        case ::SHADING_PATCH_TYPE : {
                            float const * c = getAdaptiveColor(node);
                            memcpy(col, c, 3 * sizeof(float));
                        } break;
                        case ::SHADING_PATCH_COORD : {
                            float c[3] = {s, t, 0.0f};
                            memcpy(col, c, 3 * sizeof(float));
                        } break;
                        case ::SHADING_PATCH_NORMAL : {
                            memcpy(col, norm, 3 * sizeof(float));
                        } break;
                        default:

                            break;
                    }
                }

                //memcpy(pos, c, 3 * sizeof(float));
                //memcpy(norm, n, 3 * sizeof(float));
            }
        }
    }

    createTessMesh(charmap, tessFactor, positions, normals, colors);

#else /* TESS_DOMAIN */

    typedef Far::Characteristic Char;
    typedef Far::Characteristic::Node Node;
    typedef Far::Characteristic::NodeDescriptor NodeDesc;

    std::vector<float> positions,
                       normals,
                       colors;

    int ntriangles = 0;

    for (int i=0; i<nchars; ++i) {

        Char const & ch = charmap->GetCharacteristic(i);

        // interpolate vertices
        float wP[20], wDs[20], wDt[20];

        int ncount=0;
        Node node = ch.GetTreeNode(0);
        while (node.GetTreeOffset() < ch.GetTreeSize()) {

            NodeDesc desc = node.GetDescriptor();

            if (desc.GetType()==Char::NODE_REGULAR ||
                desc.GetType()==Char::NODE_END) {

                Far::ConstIndexArray supportIndices = node.GetSupportIndices();

                for (int y=0; y<tessFactor; ++y) {
                    for (int x=0; x<tessFactor; ++x) {

                        // compute basis weights at location (s,t)
                        float s = (float)x / (float)(tessFactor-1),
                              t = (float)y / (float)(tessFactor-1);

                        ch.EvaluateBasis(node, s, t, wP, wDs, wDt);

                        // interpolate support points with basis weights
                        LimitFrame limit;
                        limit.Clear();
                        for (int k=0; k<supportIndices.size(); ++k) {
                            assert(supportIndices[k]<supportsBuffer.size());
                            Vertex const & support = supportsBuffer[supportIndices[k]];
                            limit.AddWithWeight(support, wP[k], wDs[k], wDt[k]);
                        }

                        float n[3] = { 3.3f, 3.3f, 3.3f };
                        cross(n, limit.deriv1, limit.deriv2 );

                        positions.push_back(limit.point[0]);
                        positions.push_back(limit.point[1]);
                        positions.push_back(limit.point[2]+ncount*0.25f);

                        normals.push_back(n[0]);
                        normals.push_back(n[1]);
                        normals.push_back(n[2]);

                        float const * c = g_palette[ncount % 7];
                        //float c[3] = { s, 0.1f, t };
                        colors.push_back(c[0]);
                        colors.push_back(c[1]);
                        colors.push_back(c[2]);
                    }
                }

                ntriangles += 2 * (tessFactor-1) * (tessFactor-1);

                ++ncount;
            }
            ++node;
        }
    }

    createTessMesh(charmap, tessFactor, positions, normals, colors, true);
#endif /* TESS_DOMAIN */

    delete refiner;
}

#else /* CREATE_SHAPE_TESS */

static void
createMesh(ShapeDesc const & shapeDesc, int maxlevel=3) {

    GLMesh::Topology topo = GLMesh::Topology::Cube();

    delete g_tessMesh;

    g_tessMesh = new GLMesh(topo);
}
#endif /* CREATE_SHAPE_TESS */

//------------------------------------------------------------------------------
static void
rebuildMeshes() {

    if (not g_font) {
        g_font = new GLFont(g_hud.GetFontTexture());
    }
    g_font->Clear();

    createMesh(g_shapes[ g_currentShape ], g_level);
}

//------------------------------------------------------------------------------
static void
display() {

    Stopwatch s;
    s.Start();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, g_width, g_height);
    g_hud.FillBackground();

    // prepare view matrix
    double aspect = g_width/(double)g_height;
    identity(g_transformData.ModelViewMatrix);
    translate(g_transformData.ModelViewMatrix, -g_pan[0], -g_pan[1], -g_dolly);
    rotate(g_transformData.ModelViewMatrix, g_rotate[1], 1, 0, 0);
    rotate(g_transformData.ModelViewMatrix, g_rotate[0], 0, 1, 0);
    rotate(g_transformData.ModelViewMatrix, -90, 1, 0, 0);
    translate(g_transformData.ModelViewMatrix,
              -g_center[0], -g_center[1], -g_center[2]);
    perspective(g_transformData.ProjectionMatrix,
                45.0f, (float)aspect, 0.1f, 500.0f);
    multMatrix(g_transformData.ModelViewProjectionMatrix,
               g_transformData.ModelViewMatrix,
               g_transformData.ProjectionMatrix);

    glEnable(GL_DEPTH_TEST);

    s.Stop();

    float drawCpuTime = float(s.GetElapsed() * 1000.0f);

    glBindVertexArray(0);

    glUseProgram(0);

    // draw the control mesh
    GLuint vbo = g_controlMeshVerts->BindVBO();
    int stride = g_controlMeshVerts->GetNumElements();
    g_controlMeshDisplay.Draw(vbo, 3*sizeof(float),
                              g_transformData.ModelViewProjectionMatrix);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Update and bind transform state ---------------------
    if (! g_transformUB) {
        glGenBuffers(1, &g_transformUB);
        glBindBuffer(GL_UNIFORM_BUFFER, g_transformUB);
        glBufferData(GL_UNIFORM_BUFFER, sizeof(g_transformData), NULL, GL_STATIC_DRAW);
    };
    glBindBuffer(GL_UNIFORM_BUFFER, g_transformUB);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(g_transformData), &g_transformData);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    // Update and bind lighting state ----------------------
    struct Lighting {
        struct Light {
            float position[4];
            float ambient[4];
            float diffuse[4];
            float specular[4];
        } lightSource[2];
    } lightingData = {
       {{  { 0.5,  0.2f, 1.0f, 0.0f },
           { 0.1f, 0.1f, 0.1f, 1.0f },
           { 0.7f, 0.7f, 0.7f, 1.0f },
           { 0.8f, 0.8f, 0.8f, 1.0f } },

         { { -0.8f, 0.4f, -1.0f, 0.0f },
           {  0.0f, 0.0f,  0.0f, 1.0f },
           {  0.5f, 0.5f,  0.5f, 1.0f },
           {  0.8f, 0.8f,  0.8f, 1.0f } }}
    };
    if (! g_lightingUB) {
        glGenBuffers(1, &g_lightingUB);
        glBindBuffer(GL_UNIFORM_BUFFER, g_lightingUB);
        glBufferData(GL_UNIFORM_BUFFER, sizeof(lightingData), NULL, GL_STATIC_DRAW);
    };
    glBindBuffer(GL_UNIFORM_BUFFER, g_lightingUB);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(lightingData), &lightingData);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    // Draw stuff ------------------------------------------
    bool wireframe = g_displayStyle == DISPLAY_STYLE_WIRE;
    g_tessMesh->Draw(g_transformUB, g_lightingUB, wireframe);

    // draw 3D strings (if any)
    if (g_font) {
        g_font->Draw(g_transformUB);
    }

    // Draw hud --------------------------------------------

    if (g_hud.IsVisible()) {

        g_fpsTimer.Stop();
        double fps = 1.0/g_fpsTimer.GetElapsed();
        g_fpsTimer.Start();

        // display node data
        if (g_charmap && g_currentCharIndex>=0 && g_currentNodeIndex>=0) {

            static char * nodeTypes[4] =
                { "NODE_REGULAR", "NODE_RECURSIVE", "NODE_TERMINAL", "NODE_END" };

            Far::Characteristic::NodeDescriptor desc =
                g_currentNode.GetDescriptor();

            char const * typeName = nodeTypes[desc.GetType()];


            int y = 150;
            switch (desc.GetType()) {
                case Far::Characteristic::NODE_REGULAR :
                case Far::Characteristic::NODE_END : {
                    g_hud.DrawString(g_width/2-200, y,
                        "char=%d node=%d type=%s depth=%d nonquad=%d u=%d v=%d",
                            g_currentCharIndex, g_currentNodeIndex, typeName,
                               desc.GetDepth(), desc.NonQuadRoot(), desc.GetU(), desc.GetV());
                } break;
                case Far::Characteristic::NODE_RECURSIVE : {
                    g_hud.DrawString(g_width/2-200, y, "char=%d node=%d type=%s",
                        g_currentCharIndex, g_currentNodeIndex, typeName);
                } break;
                case Far::Characteristic::NODE_TERMINAL : {
                    g_hud.DrawString(g_width/2-200, y,
                        "char=%d node=%d type=%s depth=%d nonquad=%d evIndex=%d u=%d v=%d",
                            g_currentCharIndex, g_currentNodeIndex, typeName,
                                desc.GetDepth(), desc.NonQuadRoot(), desc.GetEvIndex(), desc.GetU(), desc.GetV());
                } break;
                default:
                    assert(0);
            }
        }

        static char const * schemeNames[3] = { "BILINEAR", "CATMARK", "LOOP" };

        g_hud.DrawString(10, -120, "Scheme     : %s", schemeNames[g_shapes[g_currentShape].scheme]);
        g_hud.DrawString(10, -100, "Tess (+,-) : %d", g_tessLevel);
        g_hud.DrawString(10, -20,  "FPS        : %3.1f", fps);

        g_hud.Flush();
    }
    glFinish();

    //checkGLErrors("display leave");
}

//------------------------------------------------------------------------------
static void
callbackDisplayStyle(int b) {
    g_displayStyle = b;
}

static void
callbackModel(int m) {
    if (m < 0)
        m = 0;
    if (m >= (int)g_shapes.size())
        m = (int)g_shapes.size() - 1;
    g_currentShape = m;
    rebuildMeshes();
}

static void
callbackLevel(int l) {
    g_level = l;
    rebuildMeshes();
}

static void
callbackEndCap(int endCap) {
    g_endCap = (Far::EndCapType)endCap;
    rebuildMeshes();
}

static void
callbackShadingMode(int b) {
    g_shadingMode = b;
    rebuildMeshes();
}

static void
callbackFontScale(float value, int) {

    g_font->SetFontScale(value);
}

enum HudCheckBox { kHUD_CB_DISPLAY_CONTROL_MESH_EDGES,
                   kHUD_CB_DISPLAY_CONTROL_MESH_VERTS,
                   kHUD_CB_DISPLAY_NODE_IDS,
                   kHUD_CB_USE_TERMINAL_NODES,
                  };


static void
callbackCheckBox(bool checked, int button) {

    switch (button) {
        case kHUD_CB_DISPLAY_NODE_IDS: {
                g_DrawNodeIDs = checked;
                rebuildMeshes();
            } break;
        case kHUD_CB_DISPLAY_CONTROL_MESH_EDGES:
            g_controlMeshDisplay.SetEdgesDisplay(checked);
            break;
        case kHUD_CB_DISPLAY_CONTROL_MESH_VERTS:
            g_controlMeshDisplay.SetVerticesDisplay(checked);
            break;
        case kHUD_CB_USE_TERMINAL_NODES: {
                g_useTerminalNodes = checked;
                rebuildMeshes();
            } break;
        default:
            break;
    }
}


//------------------------------------------------------------------------------
static void
fitFrame() {

    g_pan[0] = g_pan[1] = 0;
    g_dolly = g_size;
}

//------------------------------------------------------------------------------
static void
motion(GLFWwindow *, double dx, double dy) {
    int x=(int)dx, y=(int)dy;

    if (g_hud.MouseCapture()) {
        // check gui
        g_hud.MouseMotion(x, y);
    } else if (g_mbutton[0] && !g_mbutton[1] && !g_mbutton[2]) {
        // orbit
        g_rotate[0] += x - g_prev_x;
        g_rotate[1] += y - g_prev_y;
    } else if (!g_mbutton[0] && !g_mbutton[1] && g_mbutton[2]) {
        // pan
        g_pan[0] -= g_dolly*(x - g_prev_x)/g_width;
        g_pan[1] += g_dolly*(y - g_prev_y)/g_height;
    } else if ((g_mbutton[0] && !g_mbutton[1] && g_mbutton[2]) or
               (!g_mbutton[0] && g_mbutton[1] && !g_mbutton[2])) {
        // dolly
        g_dolly -= g_dolly*0.01f*(x - g_prev_x);
        if(g_dolly <= 0.01) g_dolly = 0.01f;
    }

    g_prev_x = x;
    g_prev_y = y;
}


//------------------------------------------------------------------------------
static void
mouse(GLFWwindow *, int button, int state, int /* mods */) {

    if (state == GLFW_RELEASE)
        g_hud.MouseRelease();

    if (button == 0 && state == GLFW_PRESS && g_hud.MouseClick(g_prev_x, g_prev_y))
        return;

    if (button < 3) {
        g_mbutton[button] = (state == GLFW_PRESS);
    }
}

//------------------------------------------------------------------------------
static void
reshape(GLFWwindow *, int width, int height) {

    g_width = width;
    g_height = height;

    int windowWidth = g_width, windowHeight = g_height;

    // window size might not match framebuffer size on a high DPI display
    glfwGetWindowSize(g_window, &windowWidth, &windowHeight);

    g_hud.Rebuild(windowWidth, windowHeight, width, height);
}

//------------------------------------------------------------------------------
void windowClose(GLFWwindow*) {
    g_running = false;
}


//------------------------------------------------------------------------------
static void
keyboard(GLFWwindow *, int key, int /* scancode */, int event, int mods) {

    if (event == GLFW_RELEASE) return;
    if (g_hud.KeyDown(tolower(key))) return;

    switch (key) {
        case 'Q': g_running = 0; break;
        case 'F': fitFrame(); break;

        case '=':  {
            g_tessLevel+=5;
            rebuildMeshes();
        } break;
        case '-': {
            g_tessLevel = std::max(g_tessLevelMin, g_tessLevel-5);
            rebuildMeshes();
        } break;

        case '[': {
            if (mods==GLFW_MOD_SHIFT) {
                --g_currentCharIndex;
                g_currentNodeIndex=0;
            } else {
                --g_currentNodeIndex;
            }
            rebuildMeshes();
        } break;

        case ']': {
            if (mods==GLFW_MOD_SHIFT) {
                ++g_currentCharIndex;
                g_currentNodeIndex=0;
            } else {
                ++g_currentNodeIndex;
            }
            rebuildMeshes();
        } break;

        case GLFW_KEY_ESCAPE: g_hud.SetVisible(!g_hud.IsVisible()); break;
    }
}

//------------------------------------------------------------------------------
static void
initHUD() {

    int windowWidth = g_width, windowHeight = g_height;
    int frameBufferWidth = g_width, frameBufferHeight = g_height;

    // window size might not match framebuffer size on a high DPI display
    glfwGetWindowSize(g_window, &windowWidth, &windowHeight);
    glfwGetFramebufferSize(g_window, &frameBufferWidth, &frameBufferHeight);

    g_hud.Init(windowWidth, windowHeight, frameBufferWidth, frameBufferHeight);

    g_hud.AddCheckBox("Control edges (H)", g_controlMeshDisplay.GetEdgesDisplay(),
        10, 10, callbackCheckBox, kHUD_CB_DISPLAY_CONTROL_MESH_EDGES, 'h');
    g_hud.AddCheckBox("Control vertices (J)", g_controlMeshDisplay.GetVerticesDisplay(),
        10, 30, callbackCheckBox, kHUD_CB_DISPLAY_CONTROL_MESH_VERTS, 'j');

    g_hud.AddCheckBox("Terminal Nodes (T)", g_useTerminalNodes==1,
        10, 50, callbackCheckBox, kHUD_CB_USE_TERMINAL_NODES, 't');

    g_hud.AddCheckBox("Node IDs", g_DrawNodeIDs!=0,
        10, 70, callbackCheckBox, kHUD_CB_DISPLAY_NODE_IDS);

    int endcap_pulldown = g_hud.AddPullDown(
        "End cap (E)", 10, 230, 200, callbackEndCap, 'e');
    //g_hud.AddPullDownButton(endcap_pulldown, "None",
    //    Far::ENDCAP_NONE, g_endCap == Far::ENDCAP_NONE);
    //g_hud.AddPullDownButton(endcap_pulldown, "Bilinear",
    //    Far::ENDCAP_BILINEAR_BASIS, g_endCap == Far::ENDCAP_BILINEAR_BASIS);
    g_hud.AddPullDownButton(endcap_pulldown, "BSpline",
        Far::ENDCAP_BSPLINE_BASIS, g_endCap == Far::ENDCAP_BSPLINE_BASIS);
    g_hud.AddPullDownButton(endcap_pulldown, "GregoryBasis",
        Far::ENDCAP_GREGORY_BASIS, g_endCap == Far::ENDCAP_GREGORY_BASIS);
    //g_hud.AddPullDownButton(endcap_pulldown, "LegacyGregory",
    //    Far::ENDCAP_LEGACY_GREGORY, g_endCap == Far::ENDCAP_LEGACY_GREGORY);

    int displaystyle_pulldown = g_hud.AddPullDown(
        "DisplayStyle (W)", 200, 10, 250, callbackDisplayStyle, 'w');
    g_hud.AddPullDownButton(displaystyle_pulldown, "Wire",
        DISPLAY_STYLE_WIRE, g_displayStyle == DISPLAY_STYLE_WIRE);
    g_hud.AddPullDownButton(displaystyle_pulldown, "Shaded",
        DISPLAY_STYLE_SHADED, g_displayStyle == DISPLAY_STYLE_SHADED);
    //g_hud.AddPullDownButton(displaystyle_pulldown, "Wire+Shaded",
    //    DISPLAY_STYLE_WIRE_ON_SHADED, g_displayStyle == DISPLAY_STYLE_WIRE_ON_SHADED);

    int shading_pulldown = g_hud.AddPullDown(
        "Shading (C)", 200, 50, 250, callbackShadingMode, 'c');
    g_hud.AddPullDownButton(shading_pulldown, "Patch Type",
        ::SHADING_PATCH_TYPE, g_shadingMode == ::SHADING_PATCH_TYPE);
    g_hud.AddPullDownButton(shading_pulldown, "Patch Coord",
        ::SHADING_PATCH_COORD, g_shadingMode == ::SHADING_PATCH_COORD);
    g_hud.AddPullDownButton(shading_pulldown, "Patch Normal",
        ::SHADING_PATCH_NORMAL, g_shadingMode == ::SHADING_PATCH_NORMAL);

    g_hud.AddSlider("Font Scale", 0.0f, 0.1f, 0.01f,
                    -800, -50, 100, false, callbackFontScale, 0);

    for (int i = 1; i < 11; ++i) {
        char level[16];
        sprintf(level, "Lv. %d", i);
        g_hud.AddRadioButton(3, level, i==g_level, 10, 310+i*20, callbackLevel, i, '0'+(i%10));
    }

    int shapes_pulldown = g_hud.AddPullDown("Shape (N)", -300, 10, 300, callbackModel, 'n');
    for (int i = 0; i < (int)g_shapes.size(); ++i) {
        g_hud.AddPullDownButton(shapes_pulldown, g_shapes[i].name.c_str(),i);
    }

    g_hud.Rebuild(windowWidth, windowHeight, frameBufferWidth, frameBufferHeight);
}

//------------------------------------------------------------------------------
static void
initGL() {
    glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
#define CULLING
#ifdef CULLING
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
#else
    glDisable(GL_CULL_FACE);
#endif
}

//------------------------------------------------------------------------------
static void
uninitGL() {
}

//------------------------------------------------------------------------------
static void
idle() {

    if (g_repeatCount != 0 and g_frame >= g_repeatCount)
        g_running = 0;
}

//------------------------------------------------------------------------------
int main(int argc, char **argv) {

    std::string str;
    bool fullscreen = false;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-f")) {
            fullscreen = true;
        } else {
            std::ifstream ifs(argv[1]);
            if (ifs) {
                std::stringstream ss;
                ss << ifs.rdbuf();
                ifs.close();
                str = ss.str();
                g_shapes.push_back(ShapeDesc(argv[1], str.c_str(), kCatmark));
            }
        }
    }

    initShapes();

    if (not glfwInit()) {
        printf("Failed to initialize GLFW\n");
        return 1;
    }

    static const char windowTitle[] = "OSD CharacteristicMap test harness";

    GLUtils::SetMinimumGLVersion(argc, argv);

    if (fullscreen) {

        g_primary = glfwGetPrimaryMonitor();

        // apparently glfwGetPrimaryMonitor fails under linux : if no primary,
        // settle for the first one in the list
        if (not g_primary) {
            int count=0;
            GLFWmonitor ** monitors = glfwGetMonitors(&count);

            if (count)
                g_primary = monitors[0];
        }

        if (g_primary) {
            GLFWvidmode const * vidmode = glfwGetVideoMode(g_primary);
            g_width = vidmode->width;
            g_height = vidmode->height;
        }
    }

    if (not (g_window=glfwCreateWindow(g_width, g_height, windowTitle,
                                       fullscreen and g_primary ? g_primary : NULL, NULL))) {
        printf("Failed to open window.\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(g_window);

    // accommocate high DPI displays (e.g. mac retina displays)
    glfwGetFramebufferSize(g_window, &g_width, &g_height);
    glfwSetFramebufferSizeCallback(g_window, reshape);

    glfwSetKeyCallback(g_window, keyboard);
    glfwSetCursorPosCallback(g_window, motion);
    glfwSetMouseButtonCallback(g_window, mouse);
    glfwSetWindowCloseCallback(g_window, windowClose);

    //GLUtils::PrintGLVersion();

#if defined(OSD_USES_GLEW)
#ifdef CORE_PROFILE
    // this is the only way to initialize glew correctly under core profile context.
    glewExperimental = true;
#endif
    if (GLenum r = glewInit() != GLEW_OK) {
        printf("Failed to initialize glew. Error = %s\n", glewGetErrorString(r));
        exit(1);
    }
#ifdef CORE_PROFILE
    // clear GL errors which was generated during glewInit()
    glGetError();
#endif
#endif

    initGL();

    glfwSwapInterval(0);

    initHUD();
    rebuildMeshes();

    GLUtils::CheckGLErrors("before loop");
    while (g_running) {
        idle();
        display();

        glfwPollEvents();
        glfwSwapBuffers(g_window);

        glFinish();
    }

    uninitGL();
    glfwTerminate();

}

