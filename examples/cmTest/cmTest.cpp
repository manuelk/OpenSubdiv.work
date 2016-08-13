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

#include <far/neighborhood.h>
#include <far/patchFaceTag.h>
#include <far/patchTableFactory.h>
#include <far/stencilTable.h>
#include <far/subdivisionPlanTable.h>
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
#include <thread>

using namespace OpenSubdiv;

enum DisplayStyle { DISPLAY_STYLE_WIRE,
                    DISPLAY_STYLE_SHADED,
                    DISPLAY_STYLE_WIRE_ON_SHADED };

enum ShadingMode {
    SHADING_PATCH_TYPE,
    SHADING_PATCH_COORD,
    SHADING_PATCH_NORMAL,
    SHADING_TREE_DEPTH,
};

int g_level = 2,
    g_shadingMode = SHADING_PATCH_TYPE,
    g_tessLevel = 5,
    g_tessLevelMin = 2,
    g_currentShape = 0, //cube = 8 square = 12 pyramid = 45 torus = 49
    g_useTopologyHashing = false,
    g_useTerminalNodes = true;


int   g_frame = 0,
      g_repeatCount = 0;

bool g_DrawVertIDs = false,
     g_DrawFaceIDs = false,
     g_DrawNodeIDs = false;

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
            node.GetCharacteristic()->GetCharacteristicMap().GetEndCapType();
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

    int pattern = 0; //desc.GetTransitionCount();

    return g_patchColors[6*patchType + pattern];
}

//------------------------------------------------------------------------------

GLFont * g_font=0;

//------------------------------------------------------------------------------
int g_currentPlanIndex = -1,
    g_currentNodeIndex = 0,
    g_currentQuadrantIndex = 0;

size_t g_currentCharmapSize = 0,
       g_currentPlansTableSize = 0;

Far::Characteristic::Node g_currentNode;

static void
createVertNumbers(Far::TopologyRefiner const & refiner,
    std::vector<Vertex> const & verts) {

    int nverts = refiner.GetLevel(0).GetNumVertices();
    static char buf[16];
    for (int i=0; i<nverts; ++i) {
        snprintf(buf, 16, "%d", i);
        g_font->Print3D(verts[i].point, buf, 1);
    }
}

static void
createFaceNumbers(Far::TopologyRefiner const & refiner,
    std::vector<Vertex> const & verts) {

    Far::TopologyLevel const & level = refiner.GetLevel(0);

    for (int face=0; face<level.GetNumFaces(); ++face) {

        Far::ConstIndexArray fverts = level.GetFaceVertices(face);

        float weight = 1.0f / fverts.size();

        Vertex center;
        center.Clear();
        for (int vert=0; vert<fverts.size(); ++vert) {
            center.AddWithWeight(verts[fverts[vert]], weight);
        }
        static char buf[16];
        snprintf(buf, 16, "%d", face);
        g_font->Print3D(center.point, buf, 2);
    }
}

static void
createNodeNumbers(Far::SubdivisionPlanTable const & plansTable,
    int planIndex, Far::Characteristic::Node node, int quadrant,
        std::vector<Vertex> const & vertexBuffer) {

    int nsupports = node.GetNumSupports();

    for (int j=0; j<nsupports; ++j) {

        // get the stencil for this support point
        Far::Characteristic::Support stencil = node.GetSupport(j, quadrant);

        Vertex support;
        support.Clear();
        for (short k=0; k<stencil.size; ++k) {

            // remap the support stencil indices, which are local
            // to the characteristic's neighborhood, to the control
            // mesh topology.
            Far::Index vertIndex =
                plansTable.GetMeshControlVertexIndex(planIndex, stencil.indices[k]);

            assert(vertIndex!=Far::INDEX_INVALID);

            support.AddWithWeight(
                vertexBuffer[vertIndex], stencil.weights[k]);
        }
        float position[3] = { support.point[0],
                              support.point[1],
                              support.point[2], };
        static char buf[16];
        snprintf(buf, 16, "%d", j);
        g_font->Print3D(position, buf, 2);
    }
}

static void
createPlanNumbers(Far::SubdivisionPlanTable const & plansTable,
    int planIndex, std::vector<Vertex> const & vertexBuffer) {

    if (g_currentNodeIndex<0) {
        return;
    }

    Far::Characteristic const * ch =
        plansTable.GetPlanCharacteristic(planIndex);

    Far::Characteristic::Node node = ch->GetTreeNode(0);

    for (int count=0; node.GetTreeOffset()<ch->GetTreeSize(); ++node, ++count) {

        if (count==g_currentNodeIndex) {

            g_currentNode = node;

            Far::Characteristic::NodeDescriptor desc = node.GetDescriptor();

            if (desc.GetType()==Far::Characteristic::NODE_TERMINAL) {
                if (g_currentQuadrantIndex!=desc.GetEvIndex()) {
                    createNodeNumbers(plansTable, planIndex, node, g_currentQuadrantIndex, vertexBuffer);
                }
            } else {
                createNodeNumbers(plansTable, planIndex, node, 0, vertexBuffer);
            }
            return;
        }
    }
    g_currentNodeIndex=0;
}

//------------------------------------------------------------------------------
static void
writeDigraph(Far::CharacteristicMap const & charmap, int charIndex) {

    static int counter=0;
    char fname[64];
    snprintf(fname, 64, "digraph.%d.dot", counter++);

    FILE * fout = fopen(fname, "w");
    if (!fout) {
        fprintf(stderr, "Could not open %s\n", fname);
    }

    char const * shapename = g_shapes[g_currentShape].name.c_str();

    if (charIndex>=0 && charIndex<charmap.GetNumCharacteristics()) {

        Far::Characteristic const * ch = charmap.GetCharacteristic(charIndex);

        bool showIndices = true,
             isSubgraph = false;
        ch->WriteTreeDigraph(fout, charIndex, showIndices, isSubgraph);
    } else {
        bool showIndices = true;
        charmap.WriteCharacteristicsDigraphs(fout, shapename, showIndices);
    }
    fclose(fout);
    fprintf(stdout, "Saved %s\n", fname);
    fflush(stdout);
}

//------------------------------------------------------------------------------
static size_t
computeCharacteristicMapSize(Far::CharacteristicMap const & charmap) {
    size_t result=0;
    for (int charIndex=0; charIndex<charmap.GetNumCharacteristics(); ++charIndex) {

        Far::Characteristic const * ch = charmap.GetCharacteristic(charIndex);
        
        result += ch->GetTreeSize() * sizeof(int);
        
        result += ch->GetSupportsSizes().size() * sizeof(short);
        result += ch->GetSupportIndices().size() * sizeof(Far::LocalIndex);
        result += ch->GetSupportWeights().size() * sizeof(float);
        result += ch->GetSupportOffsets().size() * sizeof(int);

        for (int nIndex=0; nIndex<ch->GetNumNeighborhoods(); ++nIndex) {
            Far::Neighborhood const * n = ch->GetNeighborhood(nIndex);
            result += n->GetSizeWithRemap();
        }
    }
    return result;
}

static size_t
computePlansTableSize(Far::SubdivisionPlanTable const & plansTable) {
    size_t result = 0;
    result += plansTable.GetSubdivisionPlans().size() * sizeof(Far::SubdivisionPlan);
    result += plansTable.GetControlVertices().size() * sizeof(Far::Index);
    return result;   
}

//------------------------------------------------------------------------------

GLMesh * g_tessMesh = 0;

static void
tessChar(int tessFactor, int charOffset,
    std::vector<float> & positions, std::vector<float> & normals, std::vector<float> & colors,
        float * pos, float * norm, float * col) {

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
}

//#define DO_MULTI_THREAD
static void
createTessMesh(Far::SubdivisionPlanTable const & plansTable, int nplans, int tessFactor,
    std::vector<float> & positions, std::vector<float> & normals, std::vector<float> & colors,
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
    
        int ntriangles = 2 * (tessFactor-1) * (tessFactor-1) * nplans;

        // create tess mesh

        struct GLMesh::Topology topo;
        topo.positions = new float[ntriangles * 3 * 3];
        topo.normals = new float[ntriangles * 3 * 3];
        topo.colors = new float[ntriangles * 3 * 3];
        topo.nverts = 0;

        float * pos = topo.positions,
              * norm = topo.normals,
              * col = topo.colors;

        for (int i=0, offset=0; i<nplans; ++i) {

            Far::SubdivisionPlan const & plan =
                plansTable.GetSubdivisionPlans()[i];

            if (plan.charIndex==Far::INDEX_INVALID) {
                continue;
            }

            int tf = plansTable.GetPlanCharacteristic(i)->IsNonQuadPatch() ?
                tessFactor / 2 + 1 : tessFactor;

            tessChar(tf, offset, positions, normals, colors, pos, norm, col);

            int ofs = (tf-1) * (tf-1) * 18;
            pos  += ofs;
            norm += ofs;
            col  += ofs;

            offset += tf * tf;

            topo.nverts += (tf-1) * (tf-1) * 2 * 3;
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

Far::SubdivisionPlanTable const * g_plansTable;

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

    // refine adaptively
    {
        Far::TopologyRefiner::AdaptiveOptions options(maxlevel);
        options.useSingleCreasePatch = true;
        refiner->RefineAdaptive(options);
    }

    // control mesh
    delete g_controlMeshVerts;
    int nverts = shape->GetNumVertices();
    g_controlMeshVerts = Osd::GLVertexBuffer::Create(3, nverts);
    g_controlMeshVerts->UpdateData(&shape->verts[0], 0, nverts);
    g_controlMeshDisplay.SetTopology(refiner->GetLevel(0));

    // build characteristics map 
    Far::CharacteristicMap::Options options;
    options.endCapType = g_endCap;
    options.useTerminalNode = g_useTerminalNodes;
    options.hashSize = 5000;
    Far::CharacteristicMap * charmap = new Far::CharacteristicMap(options);

    delete g_plansTable;
    g_plansTable = charmap->HashTopology(*refiner);

    g_currentCharmapSize = computeCharacteristicMapSize(*charmap);
    g_currentPlansTableSize = computePlansTableSize(*g_plansTable);

    // copy coarse vertices positions
    std::vector<Vertex> controlVerts(shape->GetNumVertices());
    for (int i=0; i<shape->GetNumVertices(); ++i) {
        float const * ptr = &shape->verts[i*3];
        memcpy(controlVerts[i].point, ptr, 3 * sizeof(float));
    }

    delete shape;

    if (g_DrawVertIDs) {
        createVertNumbers(*refiner, controlVerts);
    }
    if (g_DrawFaceIDs) {
        createFaceNumbers(*refiner, controlVerts);
    }

    // draw selected node data
    {
        g_currentPlanIndex = std::max(-1,
            std::min(g_currentPlanIndex, g_plansTable->GetNumPlans()-1));
        if (g_currentPlanIndex>=0) {
            createPlanNumbers(*g_plansTable, g_currentPlanIndex, controlVerts);
        }
    }

    //
    // tessellate
    //

    int const nplans = g_plansTable->GetNumPlans();

    nverts = g_tessLevel * g_tessLevel * nplans;

    std::vector<float> positions(3 * nverts),
                       normals(3 * nverts),
                       colors(3 * nverts);
    float * pos = &positions[0],
          * norm = &normals[0],
          * col = &colors[0];

    std::vector<Vertex> supports;

    // interpolate limits

    float wP[20], wDs[20], wDt[20];

    for (int planIndex=0; planIndex<nplans; ++planIndex) {

        if (g_plansTable->PlanIsHole(planIndex)) {
            continue;
        }

        Far::Characteristic const * ch =
            g_plansTable->GetPlanCharacteristic(planIndex);

        int const tessFactor = ch->IsNonQuadPatch() ? g_tessLevel / 2 + 1 : g_tessLevel;

        // interpolate vertices
        for (int y=0; y<tessFactor; ++y) {
            for (int x=0; x<tessFactor; ++x, pos+=3, norm+=3, col+=3) {

                // compute basis weights at location (s,t)
                float s = (float)x / (float)(tessFactor-1),
                      t = (float)y / (float)(tessFactor-1);

                unsigned char quadrant=0;

                Far::Characteristic::Node node =
                    ch->EvaluateBasis(s, t, wP, wDs, wDt, &quadrant);

                //Far::Characteristic::NodeType type = node.GetDescriptor().GetType();
                
                //
                // evaluate supports from characteristic stencils
                //

                // note : the "better way" to do this would be to build a list
                // of all the nodes that contain samples, evaluate the supports
                // and then evaluate all limits. This would save a lot of
                // redundant support points evaluations (XXXX should think about
                // adding this as a far class)

                int nsupports = node.GetNumSupports();

                // this resize should be mostly "free"
                // XXXX manuelk we could allocate correctly if CharacteristicMap
                // stored the max size of all characteristics supports
                supports.resize(nsupports);

                for (int j=0; j<nsupports; ++j) {
                
                    // get the stencil for this support point
                    Far::Characteristic::Support stencil = node.GetSupport(j,quadrant);

                    supports[j].Clear();
                    for (short k=0; k<stencil.size; ++k) {

                         // remap the support stencil indices, which are local
                         // to the characteristic's neighborhood, to the control
                         // mesh topology.
                         Far::Index vertIndex =
                             g_plansTable->GetMeshControlVertexIndex(planIndex, stencil.indices[k]);

                         supports[j].AddWithWeight(
                             controlVerts[vertIndex], stencil.weights[k]);
                    }

                }

                //
                // limit points : interpolate support points with basis weights
                //

                LimitFrame limit;
                limit.Clear();
                for (int j=0; j<nsupports; ++j) {
                    limit.AddWithWeight(supports[j], wP[j], wDs[j], wDt[j]);
                }

                memcpy(pos, limit.point, 3 * sizeof(float));

                // normal
                cross(norm, limit.deriv1, limit.deriv2 );
                normalize(norm);

                Far::Characteristic::NodeDescriptor desc = node.GetDescriptor();

                // color
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
                    case ::SHADING_TREE_DEPTH : {
                        float depth = desc.GetDepth() * 0.1f;
                        float c[3] = { depth, 0.0f, 1.0f - depth };
                        memcpy(col, c, 3 * sizeof(float));
                    } break;
                    default:

                        break;
                }

                bool nodeSelected = false;
                if (g_currentPlanIndex>=0 && g_currentPlanIndex==planIndex &&
                    g_currentNodeIndex>=0 && 
                    g_currentNode==node) {
                    nodeSelected = true;
                }

                if (nodeSelected) {
                    static float quadColors[4][3] = {{ 255.0f,    0.0f,   0.0f },
                                                     {   0.0f,  255.0f,   0.0f },
                                                     {   0.0f,    0.0f, 255.0f },
                                                     {  255.0f, 255.0f,   0.0f }};

                    if (desc.GetType()==Far::Characteristic::NODE_TERMINAL) {
                        memcpy(col, quadColors[quadrant], 3 * sizeof(float));
                    } else {
                        //memcpy(col, selColor, 3 * sizeof(float));
                        col[0] *= 2.0f;
                        col[1] *= 2.0f;
                        col[2] *= 2.0f;
                    }
                }
            }
        }
    }

    createTessMesh(*g_plansTable, nplans, g_tessLevel, positions, normals, colors);

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

    if (! g_font) {
        g_font = new GLFont(g_hud.GetFontTexture());
    }
    g_font->Clear();

    createMesh(g_shapes[g_currentShape], g_level);
}


static char const *
formatMemorySize(size_t memSize) {

    static char _buf[20];

    if (memSize > 1024*1024*1024) {
        snprintf(_buf, 20, "%.3g %s", memSize / (1024.f*1024.f*1024.f), "Gb");
    } else if (memSize > 1024*1024) {
        snprintf(_buf, 20, "%.3g %s", memSize / (1024.f*1024.f), "Mb");
    } else if (memSize > 1024) {
        snprintf(_buf, 20, "%.3g %s", memSize / (1024.f), "Kb");
    } else {
        snprintf(_buf, 20, "%.3g %s", (float)memSize, "b");
    }
    return _buf;
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
#define SPECULAR
#ifdef SPECULAR           
           { 0.8f, 0.8f, 0.8f, 1.0f } },
#else
           { 0.0f, 0.0f, 0.0f, 1.0f } },
#endif

         { { -0.8f, 0.4f, -1.0f, 0.0f },
           {  0.0f, 0.0f,  0.0f, 1.0f },
           {  0.5f, 0.5f,  0.5f, 1.0f },
#ifdef SPECULAR           
           {  0.8f, 0.8f,  0.8f, 1.0f } }}
#else
           { 0.0f, 0.0f, 0.0f, 1.0f } }}
#endif
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
        if (g_plansTable && g_currentPlanIndex>=0 && g_currentNodeIndex>=0) {

            static char * nodeTypes[4] =
                { "NODE_REGULAR", "NODE_RECURSIVE", "NODE_TERMINAL", "NODE_END" };

            Far::Characteristic::NodeDescriptor desc =
                g_currentNode.GetDescriptor();

            char const * typeName = nodeTypes[desc.GetType()];

            int x = g_width/2-300, y = 200;
            switch (desc.GetType()) {
                case Far::Characteristic::NODE_REGULAR : {
                    float sharp = desc.SingleCrease() ? g_currentNode.GetSharpness() : 0.0f;
                    g_hud.DrawString(x, y,
                        "plan=%d node=%d type=%s depth=%d nonquad=%d singleCrease=%d sharp=%f u=%d v=%d",
                            g_currentPlanIndex, g_currentNodeIndex, typeName,
                               desc.GetDepth(), desc.NonQuadRoot(), desc.SingleCrease(), sharp, desc.GetU(), desc.GetV());
                } break;
                case Far::Characteristic::NODE_END : {
                    g_hud.DrawString(x, y,
                        "plan=%d node=%d type=%s depth=%d nonquad=%d u=%d v=%d",
                            g_currentPlanIndex, g_currentNodeIndex, typeName,
                               desc.GetDepth(), desc.NonQuadRoot(), desc.GetU(), desc.GetV());
                } break;
                case Far::Characteristic::NODE_RECURSIVE : {
                    g_hud.DrawString(x, y, "plan=%d node=%d type=%s",
                        g_currentPlanIndex, g_currentNodeIndex, typeName);
                } break;
                case Far::Characteristic::NODE_TERMINAL : {
                    g_hud.DrawString(x, y,
                        "plan=%d node=%d quadrant=%d type=%s depth=%d nonquad=%d evIndex=%d u=%d v=%d",
                            g_currentPlanIndex, g_currentNodeIndex, g_currentQuadrantIndex, typeName,
                                desc.GetDepth(), desc.NonQuadRoot(), desc.GetEvIndex(), desc.GetU(), desc.GetV());
                } break;
                default:
                    assert(0);
            }
        }

        static char const * schemeNames[3] = { "BILINEAR", "CATMARK", "LOOP" };

        g_hud.DrawString(10, -120, "Scheme     : %s", schemeNames[g_shapes[g_currentShape].scheme]);
        g_hud.DrawString(10, -100, "Tess (+,-) : %d", g_tessLevel);
        g_hud.DrawString(10, -40,  "Triangles  : %d", g_tessMesh ? g_tessMesh->GetNumTriangles() : -1);
        g_hud.DrawString(10, -20,  "FPS        : %3.1f", fps);

        g_hud.DrawString(-280, -120, "Chars : %d (%s)",
            g_plansTable ? g_plansTable->GetCharacteristicMap().GetNumCharacteristics() : -1, formatMemorySize(g_currentCharmapSize));

        g_hud.DrawString(-280, -100, "Plans : %d (%s)",
            g_plansTable ? (int)g_plansTable->GetSubdivisionPlans().size() : 0, formatMemorySize(g_currentPlansTableSize));

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
                   kHUD_CB_DISPLAY_VERT_IDS,
                   kHUD_CB_DISPLAY_FACE_IDS,
                   kHUD_CB_DISPLAY_NODE_IDS,
                   kHUD_CB_USE_TOPOLOGY_HASHING,
                   kHUD_CB_USE_TERMINAL_NODES,
                  };


static void
callbackCheckBox(bool checked, int button) {

    switch (button) {
        case kHUD_CB_DISPLAY_NODE_IDS: g_DrawNodeIDs = checked; break;
        case kHUD_CB_DISPLAY_VERT_IDS: g_DrawVertIDs = checked; break;
        case kHUD_CB_DISPLAY_FACE_IDS: g_DrawFaceIDs = checked; break;

        case kHUD_CB_DISPLAY_CONTROL_MESH_EDGES:
            g_controlMeshDisplay.SetEdgesDisplay(checked);
            break;
        case kHUD_CB_DISPLAY_CONTROL_MESH_VERTS:
            g_controlMeshDisplay.SetVerticesDisplay(checked);
            break;

        case kHUD_CB_USE_TOPOLOGY_HASHING: g_useTopologyHashing = checked; break;
        case kHUD_CB_USE_TERMINAL_NODES: g_useTerminalNodes = checked; break;
        default:
            break;
    }

    rebuildMeshes();
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
    } else if ((g_mbutton[0] && !g_mbutton[1] && g_mbutton[2]) ||
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

        case 'D': writeDigraph(g_plansTable->GetCharacteristicMap(),
            mods==GLFW_MOD_SHIFT ? -1 : g_currentPlanIndex);
            break;

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
                --g_currentPlanIndex;
                g_currentNodeIndex=0;
            } else if (mods==GLFW_MOD_CONTROL) {
                g_currentQuadrantIndex = (g_currentQuadrantIndex+3)%4;
            } else {
                --g_currentNodeIndex;
            }
            rebuildMeshes();
        } break;

        case ']': {
            if (mods==GLFW_MOD_SHIFT) {
                ++g_currentPlanIndex;
                g_currentNodeIndex=0;
            } else if (mods==GLFW_MOD_CONTROL) {
                g_currentQuadrantIndex = (g_currentQuadrantIndex+1)%4;
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

    g_hud.AddCheckBox("Topology Hashing", g_useTopologyHashing==1,
        10, 70, callbackCheckBox, kHUD_CB_USE_TOPOLOGY_HASHING);

    g_hud.AddCheckBox("Vert IDs", g_DrawVertIDs!=0,
        10, 120, callbackCheckBox, kHUD_CB_DISPLAY_VERT_IDS);

    g_hud.AddCheckBox("Face IDs", g_DrawFaceIDs!=0,
        10, 140, callbackCheckBox, kHUD_CB_DISPLAY_FACE_IDS);

    g_hud.AddCheckBox("Node IDs", g_DrawNodeIDs!=0,
        10, 180, callbackCheckBox, kHUD_CB_DISPLAY_NODE_IDS);

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
    g_hud.AddPullDownButton(shading_pulldown, "Tree Depth",
        ::SHADING_TREE_DEPTH, g_shadingMode == ::SHADING_TREE_DEPTH);

    g_hud.AddSlider("Font Scale", 0.0f, 0.1f, 0.01f,
                    -800, -50, 100, false, callbackFontScale, 0);

    for (int i = 1; i < 11; ++i) {
        char level[16];
        sprintf(level, "Lv. %d", i);
        g_hud.AddRadioButton(3, level, i==g_level, 10, 310+i*20, callbackLevel, i, '0'+(i%10));
    }

    int shapes_pulldown = g_hud.AddPullDown("Shape (N)", -300, 10, 300, callbackModel, 'n');
    for (int i = 0; i < (int)g_shapes.size(); ++i) {
        g_hud.AddPullDownButton(shapes_pulldown, g_shapes[i].name.c_str(), i, i==g_currentShape);
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

    if (g_repeatCount != 0 && g_frame >= g_repeatCount)
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

    if (! glfwInit()) {
        printf("Failed to initialize GLFW\n");
        return 1;
    }

    static const char windowTitle[] = "OSD CharacteristicMap test harness";

    GLUtils::SetMinimumGLVersion(argc, argv);

    if (fullscreen) {

        g_primary = glfwGetPrimaryMonitor();

        // apparently glfwGetPrimaryMonitor fails under linux : if no primary,
        // settle for the first one in the list
        if (! g_primary) {
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

    if (! (g_window=glfwCreateWindow(
        g_width, g_height, windowTitle, fullscreen && g_primary ? g_primary : NULL, NULL))) {
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

