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
#include "./tessUtils.h"
#include "./svg.h"

#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>

#if _MSC_VER
    #define snprintf _snprintf
#endif



using namespace OpenSubdiv;

enum DisplayStyle { DISPLAY_STYLE_WIRE,
                    DISPLAY_STYLE_SHADED,
                    DISPLAY_STYLE_WIRE_ON_SHADED };

enum ShadingMode {
    SHADING_PATCH_TYPE,
    SHADING_PATCH_COORD,
    SHADING_PATCH_NORMAL,
    SHADING_TREE_DEPTH,
    SHADING_TESS_FACTORS,
};

SpacingMode g_spacingMode = FRACTIONAL_ODD;

int g_level = 3,
    g_dynamicLevel = 10,
    g_shadingMode = SHADING_PATCH_TYPE,
    g_tessLevel = 5,
    g_tessLevelMin = 1,
    g_currentShape = 0, //cube = 8 square = 12 pyramid = 45 torus = 49
    g_useTerminalNodes = true,
    g_useDynamicIsolation = true,
    g_singleCreasePatch = 1,
    g_smoothCornerPatch = 0,
    g_infSharpPatch = 0;

int g_frame = 0,
    g_repeatCount = 0;

bool g_DrawVertIDs = false,
     g_DrawFaceIDs = false,
     g_DrawNodeIDs = false,
     g_DrawTessFactors = false,
     g_saveSVG = false,
     g_dynamicTess = true;

Far::EndCapType g_endCap = Far::ENDCAP_BILINEAR_BASIS;

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

GLFont * g_font=0;

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

    void Print() const {
        printf("%f %f %f", point[0], point[1], point[2]);
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

static float g_patchColors[44][4] = {
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
        {1.0f,  0.2f,  0.4f,  1.0f},   // dynamic isolation
};

static float const *
getAdaptiveColor(Far::SubdivisionPlan::Node node, int maxLevel) {

    Far::SubdivisionPlan::NodeDescriptor desc = node.GetDescriptor();

    int patchType = 0,
        pattern = 0;
    if (desc.GetType()==Far::SubdivisionPlan::NODE_REGULAR) {

        int edgeCount =desc.GetBoundaryCount();
        switch (edgeCount) {
            case 1 : patchType = 2; break;
            case 2 : patchType = 3; break;
        };

        if (desc.SingleCrease()) {
            patchType = 1;
        }
    } else if (desc.GetType()==Far::SubdivisionPlan::NODE_END) {
        Far::EndCapType endtype =
            node.GetSubdivisionPlan()->GetTopologyMap().GetEndCapType();
        switch (endtype) {
            case Far::ENDCAP_BILINEAR_BASIS : break;
            case Far::ENDCAP_BSPLINE_BASIS : break;
            case Far::ENDCAP_GREGORY_BASIS : patchType = 6; break;
            default:
                break;
        }
    } else if (desc.GetType()==Far::SubdivisionPlan::NODE_TERMINAL) {
        patchType = 7;
        if (desc.GetDepth()>=maxLevel) {
            pattern = 1;
        }
    } else if (desc.GetType()==Far::SubdivisionPlan::NODE_RECURSIVE) {
        if (desc.GetDepth()<maxLevel) {
            Far::EndCapType endtype =
                node.GetSubdivisionPlan()->GetTopologyMap().GetEndCapType();
            switch (endtype) {
                case Far::ENDCAP_BILINEAR_BASIS : break;
                case Far::ENDCAP_BSPLINE_BASIS : break;
                case Far::ENDCAP_GREGORY_BASIS : patchType = 6; break;
                default:
                    break;
            }
        } else {
            patchType = 7;
            pattern = 1;
        }
    } else {
        assert(0);
    }


    return g_patchColors[6*patchType + pattern];
}

//------------------------------------------------------------------------------

#define COLLECT_STATS
#ifdef COLLECT_STATS
    #define CLEAR_STATS() \
        g_stats.Clear();

    #define INC_STAT(statname, value) \
        g_stats.statname += value;

    #define SET_STAT(statname, value) \
        g_stats.statname = value;
#else
    #define CLEAR_STATS()

    #define INC_STAT(statname, value)

    #define SET_STAT(statname, value)
#endif

struct Stats {

    size_t topomapSize = 0,
           plansTableSize = 0,
           numSupportsEvaluated = 0,
           numSupportsTotal = 0;

    void Clear() {
        memset(this, 0, sizeof(Stats));
    }
} g_stats;

//------------------------------------------------------------------------------

int g_currentPlanIndex = -1,
    g_currentNodeIndex = 0,
    g_currentQuadrantIndex = 0;

Far::SubdivisionPlan::Node g_currentNode;

bool nodeIsSelected(int planIndex, Far::SubdivisionPlan::Node const & node) {
    bool selected = false;
    if (g_currentPlanIndex>=0 && g_currentPlanIndex==planIndex &&
        g_currentNodeIndex>=0 && g_currentNode==node) {
        selected = true;
    }
    return selected;
}

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
    int planIndex, Far::SubdivisionPlan::Node node, int quadrant,
        std::vector<Vertex> const & vertexBuffer) {

    int nsupports = node.GetNumSupports(quadrant, g_dynamicLevel);
//printf("nsupports = %d\n", nsupports); fflush(stdout);
    for (int i=0; i<nsupports; ++i) {

        // get the stencil for this support point
        Far::SubdivisionPlan::Support stencil = node.GetSupport(i, quadrant, g_dynamicLevel);

        Vertex support;
        support.Clear();
        for (short k=0; k<stencil.size; ++k) {

            // remap the support stencil indices, which are local
            // to the subdivision plan's neighborhood, to the control
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
        snprintf(buf, 16, "%d", i);
        g_font->Print3D(position, buf, 2);
    }
}

static void
createPlanCVNumbers(Far::SubdivisionPlanTable const & plansTable,
    int planIndex, std::vector<Vertex> const & vertexBuffer) {

    Far::ConstIndexArray const & verts = plansTable.GetPlanControlVertices(planIndex);

    for (int i=0; i<verts.size(); ++i) {

        Vertex const & vert = vertexBuffer[verts[i]];

        float position[3] = { vert.point[0],
                              vert.point[1],
                              vert.point[2], };

        static char buf[16];
        snprintf(buf, 16, "%d", i);
        g_font->Print3D(position, buf, 3);
    }
}


static void
createPlanNumbers(Far::SubdivisionPlanTable const & plansTable,
    int planIndex, std::vector<Vertex> const & vertexBuffer) {

    if (g_currentNodeIndex<0) {
        return;
    }

    Far::SubdivisionPlan const * plan =
        plansTable.GetSubdivisionPlan(planIndex);

    Far::SubdivisionPlan::Node node = plan->GetTreeNode(0);

    for (int count=0; node.GetTreeOffset()<plan->GetTreeSize(); ++node, ++count) {

        if (count==g_currentNodeIndex) {

            g_currentNode = node;

            Far::SubdivisionPlan::NodeDescriptor desc = node.GetDescriptor();

            if (desc.GetType()==Far::SubdivisionPlan::NODE_TERMINAL) {
                createNodeNumbers(plansTable, planIndex, node, g_currentQuadrantIndex, vertexBuffer);
            } else {
                createNodeNumbers(plansTable, planIndex, node, 0, vertexBuffer);
            }
            createPlanCVNumbers(plansTable, planIndex, vertexBuffer);
            return;
        }
    }

    g_currentNodeIndex=0;
}

//------------------------------------------------------------------------------
static void
writeDigraph(Far::TopologyMap const & topomap, int planIndex) {

    static int counter=0;
    char fname[64];
    snprintf(fname, 64, "digraph.%d.dot", counter++);

    FILE * fout = fopen(fname, "w");
    if (!fout) {
        fprintf(stderr, "Could not open %s\n", fname);
    }

    char const * shapename = g_shapes[g_currentShape].name.c_str();

    if (planIndex>=0 && planIndex<topomap.GetNumSubdivisionPlans()) {

        Far::SubdivisionPlan const * plan = topomap.GetSubdivisionPlan(planIndex);

        bool showIndices = true,
             isSubgraph = false;
        plan->WriteTreeDigraph(fout, planIndex, showIndices, isSubgraph);
    } else {
        bool showIndices = true;
        topomap.WriteSubdivisionPlansDigraphs(fout, shapename, showIndices);
    }
    fclose(fout);
    fprintf(stdout, "Saved %s\n", fname);
    fflush(stdout);
}

//------------------------------------------------------------------------------
static size_t
computeTopologyMapSize(Far::TopologyMap const & topomap) {
    size_t result=0;
    for (int planIndex=0; planIndex<topomap.GetNumSubdivisionPlans(); ++planIndex) {

        Far::SubdivisionPlan const * plan = topomap.GetSubdivisionPlan(planIndex);

        result += plan->GetTreeSize() * sizeof(int);

        result += plan->GetSupportsSizes().size() * sizeof(short);
        result += plan->GetSupportIndices().size() * sizeof(Far::LocalIndex);
        result += plan->GetSupportWeights().size() * sizeof(float);
        result += plan->GetSupportOffsets().size() * sizeof(int);

        for (int nIndex=0; nIndex<plan->GetNumNeighborhoods(); ++nIndex) {
            Far::Neighborhood const * n = plan->GetNeighborhood(nIndex);
            result += n->GetSizeWithRemap();
        }
    }
    return result;
}

static size_t
computePlansTableSize(Far::SubdivisionPlanTable const & plansTable) {
    size_t result = 0;
    result += plansTable.GetFacePlans().size() * sizeof(Far::FacePlan);
    result += plansTable.GetControlVertices().size() * sizeof(Far::Index);
    return result;
}



//------------------------------------------------------------------------------

inline float
roundUpEven(float x) {
//printf("round even %f -> %f\n", x, 2.0f * std::ceil(x/2.0f));
    return 2.0f * std::ceil(x/2.0f);
}

inline float
roundUpOdd(float x) {
//printf("round odd %f -> %f\n", x,  2.0f * std::ceil((x+1.0f)/2.0f)-1.0f);
    return 2.0f * std::ceil((x+1.0f)/2.0f)-1.0f;
}

struct TessFactors {
    float outerLo[4],
          outerHi[4],
          outer[4],
          inner[2];

    int isolationLevel;

    void SetDefault(float f=0.0) {
        outerLo[0]=outerLo[1]=outerLo[2]=outerLo[3]=f;
        outerHi[0]=outerHi[1]=outerHi[2]=outerHi[3]=f;
        outer[0]=outer[1]=outer[2]=outer[3]=f;
        inner[0]=inner[1]=f;
        isolationLevel=10;
    }

    // computes outer & inner from outerLo & outerHi
    void ComputeTessLevels(SpacingMode spacing) {
        float combinedOuter[4] = {
            outerLo[0] + outerHi[0],
            outerLo[1] + outerHi[1],
            outerLo[2] + outerHi[2],
            outerLo[3] + outerHi[3],
        };

        switch (spacing) {
            case FRACTIONAL_ODD: {
                for (int i=0; i<4; ++i) {
                    outer[i] = outerHi[i] > 0.0f ?
                        roundUpOdd(outerLo[i]) + roundUpOdd(outerHi[i]) :
                            combinedOuter[i];
                    combinedOuter[i] = std::max(3.0f, combinedOuter[i]);
                }
            } break;
            case FRACTIONAL_EVEN: {
                for (int i=0; i<4; ++i) {
                    outer[i] = outerHi[i] > 0.0f ?
                        roundUpEven(outerLo[i]) + roundUpEven(outerHi[i]) :
                            combinedOuter[i];
                }
            } break;
            case EQUAL: {
                for (int i=0; i<4; ++i) {
                    outerLo[i] = std::round(outerLo[i]);
                    outerHi[i] = std::round(outerHi[i]);
                    outer[i] = combinedOuter[i] = outerLo[i] + outerHi[i];
                }
            } break;
        }

        inner[0] = (combinedOuter[1]+combinedOuter[3]) * 0.5f;
        inner[1] = (combinedOuter[0]+combinedOuter[2]) * 0.5f;
    }

    void Print() const {
        printf("outerLo [%f %f %f %f]\n", outerLo[0], outerLo[1], outerLo[2], outerLo[3]);
        printf("outerHi [%f %f %f %f]\n", outerHi[0], outerHi[1], outerHi[2], outerHi[3]);
        printf("outer   [%f %f %f %f]\n", outer[0], outer[1], outer[2], outer[3]);
        printf("inner   [%f %f]\n",       inner[0], inner[1]);
        printf("isolation = %d\n",        isolationLevel);
    }
};

// returns squared vector magnitude
static float
distance(float const * a, float const * b) {
    float ab0 = b[0] - a[0],
          ab1 = b[1] - a[1],
          ab2 = b[2] - a[2];
    return sqrtf(ab0*ab0 + ab1*ab1 + ab2*ab2);
}

static float
computeProjectedSphereExtent(float const * center, float diameter) {

    float p[4] = { center[0], center[1], center[2], 1.0f };

    apply(p, g_transformData.ProjectionMatrix);

    return fabs(diameter * g_transformData.ProjectionMatrix[5] / p[3]);
}

static float
computeTessFactor(Vertex const & p0, Vertex const & p1, int tessLevel) {

    float a[4] = { p0.point[0], p0.point[1], p0.point[2], 1.0f },
          b[4] = { p1.point[0], p1.point[1], p1.point[2], 1.0f };

    // transform to camera
    apply(a, g_transformData.ModelViewMatrix);
    apply(b, g_transformData.ModelViewMatrix);

    float center[3] = { 0.5f*(a[0] + b[0]),
                        0.5f*(a[1] + b[1]),
                        0.5f*(a[2] + b[2]) };

    float diameter = distance(a, b),
          projLength = computeProjectedSphereExtent(center, diameter),
          tessFactor = std::min(64.0f,std::max(1.0f, tessLevel * projLength));

    return tessFactor;
}

static void
computeTessFactors(SpacingMode spacing,
    Far::SubdivisionPlanTable const * plansTable, std::vector<Vertex> const & verts,
        int isolationLevel, int dynamicLevel, int tessLevel, TessFactors * tf, int * nverts, int * ntris) {

    for (int planIndex=0; planIndex < plansTable->GetNumPlans(); ++planIndex) {

        tf->SetDefault(0.0);

        if (plansTable->PlanIsHole(planIndex))
            continue;

        Far::SubdivisionPlan const * plan =
            plansTable->GetSubdivisionPlan(planIndex);

        Far::ConstIndexArray indices =
            plansTable->GetPlanControlVertices(planIndex);

        bool nonquad = plan->IsNonQuadPatch();

        if (nonquad) {

            // first n entires are 0 ring, but need to be rotated by 'quadrant'
            int valence = plan->GetCoarseFaceValence(),
                quadrant = plan->GetCoarseFaceQuadrant();

            Vertex v0, v1, v2, v3;

            v0 = verts[indices[quadrant]];

            v1.Clear(); // edge forward midpoint
            v1.AddWithWeight(v0, 0.5f);
            v1.AddWithWeight(verts[indices[(quadrant+1)%valence]], 0.5f);

            v2.Clear();
            for (int i=0; i<valence; ++i) {
                v2.AddWithWeight(verts[indices[i]], 1.0f/valence);
            }

            v3.Clear(); // edge backward midpoint
            v3.AddWithWeight(verts[indices[(quadrant+valence-1)%valence]], 0.5f);
            v3.AddWithWeight(v0, 0.5f);

            float f0 = computeTessFactor(v0, v1, tessLevel),
                  f1 = computeTessFactor(v1, v2, tessLevel),
                  f2 = computeTessFactor(v2, v3, tessLevel),
                  f3 = computeTessFactor(v3, v0, tessLevel);

            tf->outerLo[0] = f0; tf->outerHi[0] = 0.0f;
            tf->outerLo[1] = f1; tf->outerHi[1] = 0.0f;
            tf->outerLo[2] = f2, tf->outerHi[2] = 0.0f;
            tf->outerLo[3] = f3; tf->outerHi[3] = 0.0f;
        } else {
            for (int i=0; i<4; ++i) {

                Vertex v0 = verts[indices[i]],
                       v1 = verts[indices[(i+1)%4]];

                Vertex ev; // mid-point
                ev.Clear();
                ev.AddWithWeight(v0, 0.5f);
                ev.AddWithWeight(v1, 0.5f);

                tf->outerLo[i] = computeTessFactor(v0, ev, tessLevel);
                tf->outerHi[i] = computeTessFactor(ev, v1, tessLevel);
            }
        }

        tf->ComputeTessLevels(spacing);
        if (tf->inner[0]==0.0f || tf->inner[1]==0.0f) {
            tf->SetDefault(2.0f);
        }

        // dynamic isolation level
        {
            float tessMax = 0;
            for (int i=0; i<4; ++i) {
                tessMax = std::max(tessMax, tf->outerLo[i] + tf->outerHi[i]);
            }
            int level = (int)tessMax,
                levelCap = std::min(isolationLevel, dynamicLevel);
            tf->isolationLevel = std::min(level, levelCap);
        }
//printf("tessFactors plan=%d valence=%d\n", planIndex, plan->GetCoarseFaceValence());
//tf->Print();
//fflush(stdout);

        PatchInfo pi;
        pi.set(QUAD, spacing, tf->inner, tf->outer);

        *nverts += pi.patch_verts;
        *ntris += pi.patch_prims;

        ++tf;
    }
}

static float
computeTessFractionalSplit(
    SpacingMode spacing, float t, float level, float levelUp) {

    // Fractional tessellation of an edge will produce n segments where n
    // is the tessellation level of the edge (level) rounded up to the
    // nearest even or odd integer (levelUp). There will be n-2 segments of
    // equal length (dx1) and two additional segments of equal length (dx0)
    // that are typically shorter than the other segments. The two additional
    // segments should be placed symmetrically on opposite sides of the
    // edge (offset).

    float base, offset;
    switch (spacing) {

        case FRACTIONAL_ODD: {
            if (level <= 1.0f) return t;
            base = powf(2.0f, std::floor(log2f(levelUp))),
            offset = 1.0f/(((int(2.0f*base-levelUp)/2+1) & int(base/2.0f-1.0f))+1.0f);
        } break;

        case FRACTIONAL_EVEN: {
            if (level <= 2.0f) return t;
            base = powf(2.0f,std::floor(log2f(levelUp))),
            offset = 1.0f/(int(2*base-levelUp)/2 & int(base/2.0f-1.0f));
        } break;

        case EQUAL: {
        } break;
    }

    float dx0 = (1.0f - (levelUp-level)/2.0f) / levelUp,
          dx1 = (1.0f - 2.0f*dx0) / (levelUp - 2.0f*std::ceil(dx0));

    if (t < 0.5f) {
        float x = levelUp/2.0f - std::round(t*levelUp);
        return 0.5f - (x*dx1 + int(x*offset > 1.0f) * (dx0 - dx1));
    } else if (t > 0.5f) {
        float x = std::round(t*levelUp) - levelUp/2.0f;
        return 0.5f + (x*dx1 + int(x*offset > 1.0f) * (dx0 - dx1));
    } else {
        return t;
    }
}

static float
computeTessTransitionSplit(
    SpacingMode spacing, float t, float lo, float hi) {


    switch (spacing) {

        case FRACTIONAL_ODD: {
            float loRoundUp = roundUpOdd(lo),
                  hiRoundUp = roundUpOdd(hi);
            // Convert the parametric t into a segment index along the combined edge.
            // The +1 below is to account for the extra segment produced by the
            // tessellator since the sum of two odd tess levels will be rounded
            // up by one to the next odd integer tess level.
            float ti = std::round(t * (loRoundUp + hiRoundUp + 1.0f));

            if (ti <= loRoundUp) {
                float t0 = ti / loRoundUp;
                return computeTessFractionalSplit(spacing, t0, lo, loRoundUp) * 0.5f;
            } else if (ti > (loRoundUp+1.0f)) {
                float t1 = (ti - (loRoundUp+1.0f)) / hiRoundUp;
                return computeTessFractionalSplit(spacing, t1, hi, hiRoundUp) * 0.5f + 0.5f;
            } else {
                return 0.5f;
            }
        } break;

        case FRACTIONAL_EVEN: {
            float loRoundUp = roundUpEven(lo),
                  hiRoundUp = roundUpEven(hi);
            float ti = std::round(t * (loRoundUp + hiRoundUp));
            if (ti <= loRoundUp) {
                float t0 = ti / loRoundUp;
                return computeTessFractionalSplit(spacing, t0, lo, loRoundUp) * 0.5f;
            } else {
                float t1 = (ti - loRoundUp) / hiRoundUp;
                return computeTessFractionalSplit(spacing, t1, hi, hiRoundUp) * 0.5f + 0.5f;
            }
        } break;

        case EQUAL: {
            // Convert the parametric t into a segment index along the combined edge.
            float ti = round(t * (lo + hi));

            if (ti <= lo) {
                return (ti / lo) * 0.5f;
            } else {
                return ((ti - lo) / hi) * 0.5f + 0.5f;
            }
        } break;
    }
    return -1.0f;
}

static void
computeTessParameterization(SpacingMode spacing, TessFactors const & tf, float * u, float * v) {

    assert(u && v);

    float s=*u, t=*v;
           if (s==0.0f && tf.outerHi[0]>0.0f) {
        t = computeTessTransitionSplit(spacing, t, tf.outerHi[3], tf.outerLo[3]);
    } else if (t==0.0f && tf.outerHi[0]>0.0f) {
        s = computeTessTransitionSplit(spacing, s, tf.outerLo[0], tf.outerHi[0]);
    } else if (s==1.0f && tf.outerHi[0]>0.0f) {
        t = computeTessTransitionSplit(spacing, t, tf.outerLo[1], tf.outerHi[1]);
    } else if (t==1.0f && tf.outerHi[0]>0.0f) {
        s = computeTessTransitionSplit(spacing, s, tf.outerHi[2], tf.outerLo[2]);
    }
    *u = s;
    *v = t;
}

static void
computeSupports(Far::SubdivisionPlanTable const * plansTable,
    int planIndex, int levelIndex, std::vector<Vertex> const & controlVerts, Vertex * supports);

static void
createTessFactorNumbers(Far::SubdivisionPlanTable const & plansTable,
    int planIndex, std::vector<Vertex> const & supports, TessFactors const & tf) {

    // create a 3D string by evaluating a point on the limit surface that
    // is in the middle of the edge, nudged 0.05 units towards the parametric
    // center

    static float const u[5] = { 0.5f,  0.95f, 0.5f,  0.05f, 0.5f };
    static float const v[5] = { 0.05f, 0.5f,  0.95f, 0.5f,  0.5f };

    Far::TopologyMap const & topomap = plansTable.GetTopologyMap();

    unsigned char quadrant=0;
    float wP[20], wDs[20], wDt[20];

    Far::SubdivisionPlan const * plan =
        plansTable.GetSubdivisionPlan(planIndex);

    Far::ConstIndexArray planVerts = plansTable.GetPlanControlVertices(planIndex);

    for (int i=0; i<5; ++i) {

        float s = u[i],
              t = v[i];

        Far::SubdivisionPlan::Node node =
            plan->EvaluateBasis(s, t, wP, wDs, wDt, &quadrant, tf.isolationLevel);

        int nsupports = node.GetNumSupports(quadrant);

        LimitFrame limit;
        limit.Clear();
        for (int j=0; j<nsupports; ++j) {
            Far::Index supportIndex = node.GetSupportIndex(j, quadrant, tf.isolationLevel);
            limit.AddWithWeight(supports[supportIndex], wP[j], wDs[j], wDt[j]);
        }

        static char buf[16];
        if (i<4) {
            //float tf = tessFactors[planIndex].outerLo[i] + tessFactors[planIndex].outerHi[i];
            snprintf(buf, 16, "%c=%f",'A'+i, tf.outer[i]);
            g_font->Print3D(limit.point, buf, 4);
        } else {
            snprintf(buf, 16, "%d", tf.isolationLevel);
            g_font->Print3D(limit.point, buf, 7);
        }
    }
}

static void
computeTessFactorColor(
    int tessLevel, int x, int y, TessFactors const & tessFactors, float * color) {

    int edge = -1;
    if (x > y) {
        if (x < (tessLevel-y)) edge = 0;
        else                   edge = 1;
    } else {
        if (x < (tessLevel-y)) edge = 3;
        else                   edge = 2;
    }
    assert(edge>=0);
    float outerFactor = tessFactors.outerLo[edge];

    static float const nearc[3] = { 1.0f, 1.0f, 1.0f },
                       farc[3] = { 0.0f, 0.0f, 0.0f };

    float s = (outerFactor - 1.0f) / 63.f;

    color[0] = farc[0] * (1.0f - s) + nearc[0] * s;
    color[1] = farc[1] * (1.0f - s) + nearc[1] * s;
    color[2] = farc[2] * (1.0f - s) + nearc[2] * s;
}


//------------------------------------------------------------------------------

GLMesh * g_tessMesh = 0;

GLControlMeshDisplay g_controlMeshDisplay;

Osd::GLVertexBuffer * g_controlMeshVerts = 0;

Far::SubdivisionPlanTable const * g_plansTable;

#define CREATE_SHAPE_TESS
#ifdef CREATE_SHAPE_TESS

//
// apply color highlight if the node is currently selected
//
static void
applyNodeColor(int planIndex,
    Far::SubdivisionPlan::Node node, unsigned char quadrant, float * col) {

     Far::SubdivisionPlan::NodeDescriptor desc = node.GetDescriptor();

     bool nodeSelected = nodeIsSelected(planIndex, node);

     if (nodeSelected) {
         static float quadColors[4][3] = {{ 255.0f,    0.0f,   0.0f },
                                          {   0.0f,  255.0f,   0.0f },
                                          {   0.0f,    0.0f, 255.0f },
                                          {  255.0f, 255.0f,   0.0f }};

         if (desc.GetType()==Far::SubdivisionPlan::NODE_TERMINAL) {
             memcpy(col, quadColors[quadrant], 3 * sizeof(float));
         } else {
             //memcpy(col, selColor, 3 * sizeof(float));
             col[0] *= 2.0f;
             col[1] *= 2.0f;
             col[2] *= 2.0f;
         }
     }
}

//
// Evaluate all supports points for this plan.
// Depending on sampling density, we may end up using only a few of
// them. To avoid this, we could first build a list of Nodes
// (ie. sub-patches) that we actually want to sample, and then
// evaluate only those support stencils.
// note : function assumes that supports array has allocated enough verts

static void
computeSupports(Far::SubdivisionPlanTable const * plansTable,
    int planIndex, int levelIndex, std::vector<Vertex> const & controlVerts, Vertex * supports) {

    Far::SubdivisionPlan const * plan =
        plansTable->GetSubdivisionPlan(planIndex);

    int nsupports = levelIndex > 0 ?
        plan->GetNumSupports(levelIndex) : plan->GetNumSupportsTotal();
    for (int i=0; i<nsupports; ++i) {

        // get the stencil for this support point
        Far::SubdivisionPlan::Support stencil = plan->GetSupport(i);

        supports[i].Clear();
        for (short k=0; k<stencil.size; ++k) {

             // remap the support stencil indices, which are local
             // to the subdivision plan's neighborhood, to the control
             // mesh topology.
             Far::Index vertIndex =
                 plansTable->GetMeshControlVertexIndex(planIndex, stencil.indices[k]);

             supports[i].AddWithWeight(controlVerts[vertIndex], stencil.weights[k]);
        }
    }

    INC_STAT(numSupportsEvaluated, nsupports);
    INC_STAT(numSupportsTotal, plan->GetNumSupportsTotal());
}

static void
createMesh(ShapeDesc const & shapeDesc, int maxlevel=3) {

    CLEAR_STATS();

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
        options.useSingleCreasePatch = g_singleCreasePatch;
        options.useInfSharpPatch = g_infSharpPatch;
        refiner->RefineAdaptive(options);
    }

    // topology map & plans table
    Far::TopologyMap * topomap = 0;
    std::vector<Vertex> controlVerts(shape->GetNumVertices());
    {
        delete g_controlMeshVerts;
        int nverts = shape->GetNumVertices(), ntriangles = 0;
        g_controlMeshVerts = Osd::GLVertexBuffer::Create(3, nverts);
        g_controlMeshVerts->UpdateData(&shape->verts[0], 0, nverts);
        g_controlMeshDisplay.SetTopology(refiner->GetLevel(0));

        // build topology map
        Far::TopologyMap::Options options;
        options.endCapType = g_endCap;
        options.useTerminalNode = g_useTerminalNodes;
        options.useDynamicIsolation = g_useDynamicIsolation;
        options.generateLegacySharpCornerPatches = g_smoothCornerPatch;
        options.hashSize = 5000;
        topomap = new Far::TopologyMap(options);

        delete g_plansTable;
        g_plansTable = topomap->HashTopology(*refiner);

        SET_STAT(topomapSize, computeTopologyMapSize(*topomap));
        SET_STAT(plansTableSize, computePlansTableSize(*g_plansTable));

        // copy coarse vertices positions
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

    int nplans = g_plansTable->GetNumPlans(),
        nverts=0,
        ntriangles=0;

    SpacingMode spacing = g_spacingMode;

    // compute tess factos
    std::vector<TessFactors> tessFactors;
    tessFactors.resize(nplans);
    if (g_dynamicTess) {
        computeTessFactors(spacing, g_plansTable, controlVerts,
            g_level, g_dynamicLevel, g_tessLevel, &tessFactors[0], &nverts, &ntriangles);
    } else {
        nverts = g_tessLevel * g_tessLevel * nplans,
        ntriangles = 2 * std::max(1, g_tessLevel-1) * std::max(1, g_tessLevel-1) * nplans;
    }

    int size = ntriangles * 3 * 3;
    std::vector<float> positions(size),
                       normals(size),
                       colors(size);
    float * pos = &positions[0],
          * norm = &normals[0],
          * col = &colors[0];

    std::vector<Vertex> supports;
    supports.resize(topomap->GetNumMaxSupports());

    unsigned char quadrant=0;
    float wP[20], wDs[20], wDt[20];

    for (int planIndex=0; planIndex<nplans; ++planIndex) {

        if (g_plansTable->PlanIsHole(planIndex)) {
            continue;
        }

        Far::SubdivisionPlan const * plan =
            g_plansTable->GetSubdivisionPlan(planIndex);

        bool nonquad = plan->IsNonQuadPatch();

        //
        // create topology
        //

        std::vector<int> indices;
        std::vector<float> u, v;

        TessFactors const & tf = tessFactors[planIndex];

        if (g_dynamicTess) {
            // dynamic (screen-space) tessellation
            tessellate(QUAD, spacing, tf.inner, tf.outer, indices, u, v);
        } else {
            int tessLevel = nonquad ? g_tessLevel / 2 + 1 : g_tessLevel;
            tessellate(tessLevel, indices, u, v);
        }

        //
        // compute all sub-patch stencils for this plan
        //

        int isolationLevel = g_dynamicTess ? tf.isolationLevel : g_dynamicLevel;

        isolationLevel = std::min(isolationLevel, g_level);

        computeSupports(g_plansTable, planIndex, isolationLevel, controlVerts, &supports[0]);

        if (g_dynamicTess && g_DrawTessFactors) {
            createTessFactorNumbers(*g_plansTable, planIndex, supports, tf);
        }

        for (int i=0; i<(int)indices.size(); ++i, pos+=3, norm+=3, col+=3) {

            //
            // evaluate basis weights
            //

            float s = u[indices[i]],
                  t = v[indices[i]];

            computeTessParameterization(spacing, tf, &s, &t);

            Far::SubdivisionPlan::Node node =
                plan->EvaluateBasis(s, t, wP, wDs, wDt, &quadrant, isolationLevel);

            //
            // limit points : interpolate support points with basis weights
            //
            int nsupports = node.GetNumSupports(quadrant, isolationLevel);

            LimitFrame limit;
            limit.Clear();
            for (int j=0; j<nsupports; ++j) {
                Far::Index supportIndex = node.GetSupportIndex(j, quadrant, isolationLevel);
                limit.AddWithWeight(supports[supportIndex], wP[j], wDs[j], wDt[j]);
            }

            memcpy(pos, limit.point, 3 * sizeof(float));

            // normal
            cross(norm, limit.deriv1, limit.deriv2 );
            normalize(norm);

            // color
            switch (g_shadingMode) {
                case ::SHADING_PATCH_TYPE : {
                    float const * c = getAdaptiveColor(node, isolationLevel);
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
                    Far::SubdivisionPlan::NodeDescriptor desc = node.GetDescriptor();
                    float depth = desc.GetDepth() * 0.1f;
                    float c[3] = { depth, 0.0f, 1.0f - depth };
                    memcpy(col, c, 3 * sizeof(float));
                } break;
                //case ::SHADING_TESS_FACTORS: {
                //    float c[3] = { 0.0f, 0.0f, 0.0f };
                //    computeTessFactorColor(tessLevel, x, y, &tessFactors[planIndex * 6], c);
                //    memcpy(col, c, 3 * sizeof(float));
                //} break;
                default:

                    break;
            }
            applyNodeColor(planIndex, node, quadrant, col);
        }
    }

    delete g_tessMesh;
    struct GLMesh::Topology topo;
    topo.positions = &positions[0];
    topo.normals = &normals[0];
    topo.colors = &colors[0];
    topo.nverts = (int)positions.size()/3;
    g_tessMesh = new GLMesh(topo);

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


inline char const *
getNodeData(Far::SubdivisionPlan::Node const & node) {

        static char * nodeTypes[4] =
            { "NODE_REGULAR", "NODE_RECURSIVE", "NODE_TERMINAL", "NODE_END" };

        static char buffer[256];

        Far::SubdivisionPlan::NodeDescriptor desc =
            node.GetDescriptor();

        char const * typeName = nodeTypes[desc.GetType()];

        switch (desc.GetType()) {
            case Far::SubdivisionPlan::NODE_REGULAR :
            case Far::SubdivisionPlan::NODE_END : {
                float sharp = desc.SingleCrease() ? node.GetSharpness() : 0.0f;
                snprintf(buffer, 256, "type=%s nonquad=%d singleCrease=%d sharp=%f depth=%d boundary=%d u=%d v=%d",
                    nodeTypes[desc.GetType()], desc.NonQuadRoot(), desc.SingleCrease(), sharp,
                        desc.GetDepth(), desc.GetBoundaryMask(), desc.GetU(), desc.GetV());
            } break;
            case Far::SubdivisionPlan::NODE_RECURSIVE : {
                snprintf(buffer, 256, "type=%s nonquad=%d depth=%d",
                    nodeTypes[desc.GetType()], desc.NonQuadRoot(), desc.GetDepth());
            } break;
            case Far::SubdivisionPlan::NODE_TERMINAL : {
                snprintf(buffer, 256, "type=%s nonquad=%d depth=%d evIndex=%d u=%d v=%d",
                    nodeTypes[desc.GetType()], desc.NonQuadRoot(), desc.GetDepth(),
                        desc.GetEvIndex(), desc.GetU(), desc.GetV());
            } break;
        }
        return buffer;
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

Svg * svg = 0;
if (g_saveSVG)
    svg = Svg::Create("screenshot.svg");

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

if (g_saveSVG) {
    svg->SetLineWidth(0.1f);
    svg->SetLineColor(0.5f, 0.5f, 0.5f, 0.5f);
    svg->Write();
}
    // draw the control mesh
    GLuint vbo = g_controlMeshVerts->BindVBO();
    int stride = g_controlMeshVerts->GetNumElements();
    g_controlMeshDisplay.Draw(vbo, 3*sizeof(float),
                              g_transformData.ModelViewProjectionMatrix);

if (g_saveSVG) {
    svg->SetPointSize(2.5f);
    svg->SetPointColor(0.9f, 0.3f, 0.2f);
    svg->SetLineWidth(1.0f);
    svg->SetLineColor(0.463f, 0.725f, 0.0f);
    svg->Write();
    delete svg;
    g_saveSVG = false;
}

    glBindBuffer(GL_ARRAY_BUFFER, 0);


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

            Far::FacePlan const & plan = g_plansTable->GetPlan(g_currentPlanIndex);

            Far::SubdivisionPlan::NodeDescriptor desc =
                g_currentNode.GetDescriptor();

            char const * nodeData = getNodeData(g_currentNode);

            int x = g_width/2-300, y = 200;

            g_hud.DrawString(x, y, "plan=%d ch=%d ncvs=%d node=%d {%s}",
                g_currentPlanIndex, plan.planIndex, plan.numControls, g_currentNodeIndex, nodeData);
        }

        static char const * schemeNames[3] = { "BILINEAR", "CATMARK", "LOOP" };

        g_hud.DrawString(10, -120, "Scheme     : %s", schemeNames[g_shapes[g_currentShape].scheme]);
        g_hud.DrawString(10, -100, "Tess (+,-) : %d", g_tessLevel);
        g_hud.DrawString(10, -40,  "Triangles  : %d", g_tessMesh ? g_tessMesh->GetNumTriangles() : -1);
        g_hud.DrawString(10, -20,  "FPS        : %3.1f", fps);

        g_hud.DrawString(-280, -140, "Supports : %d / %d",
            g_stats.numSupportsEvaluated, g_stats.numSupportsTotal);

        g_hud.DrawString(-280, -120, "TopoMap : %d (%s)",
            g_plansTable ? g_plansTable->GetTopologyMap().GetNumSubdivisionPlans() : -1, formatMemorySize(g_stats.topomapSize));

        g_hud.DrawString(-280, -100, "PlansTable : %d (%s)",
            g_plansTable ? (int)g_plansTable->GetFacePlans().size() : 0, formatMemorySize(g_stats.plansTableSize));

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
callbackDynamicLevel(int l) {
    g_dynamicLevel = l;
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
                   kHUD_CB_DISPLAY_TESSFACTORS,
                   kHUD_CB_USE_TERMINAL_NODES,
                   kHUD_CB_USE_DYNAMIC_ISOLATION,
                   kHUD_CB_SMOOTH_CORNER_PATCH,
                   kHUD_CB_SINGLE_CREASE_PATCH,
                   kHUD_CB_INF_SHARP_PATCH,
                   kHUD_CB_DYNAMIC_TESS,
                  };


static void
callbackCheckBox(bool checked, int button) {

    switch (button) {
        case kHUD_CB_DISPLAY_NODE_IDS: g_DrawNodeIDs = checked; break;
        case kHUD_CB_DISPLAY_VERT_IDS: g_DrawVertIDs = checked; break;
        case kHUD_CB_DISPLAY_FACE_IDS: g_DrawFaceIDs = checked; break;
        case kHUD_CB_DISPLAY_TESSFACTORS: g_DrawTessFactors = checked; break;

        case kHUD_CB_DISPLAY_CONTROL_MESH_EDGES:
            g_controlMeshDisplay.SetEdgesDisplay(checked);
            break;
        case kHUD_CB_DISPLAY_CONTROL_MESH_VERTS:
            g_controlMeshDisplay.SetVerticesDisplay(checked);
            break;

        case kHUD_CB_USE_TERMINAL_NODES: g_useTerminalNodes = checked; break;
        case kHUD_CB_USE_DYNAMIC_ISOLATION: g_useDynamicIsolation = checked; break;
        case kHUD_CB_SMOOTH_CORNER_PATCH: g_smoothCornerPatch = checked; break;
        case kHUD_CB_SINGLE_CREASE_PATCH: g_singleCreasePatch = checked; break;
        case kHUD_CB_INF_SHARP_PATCH: g_infSharpPatch = checked; break;

        case kHUD_CB_DYNAMIC_TESS: g_dynamicTess = checked; break;

        default:
            break;
    }

    rebuildMeshes();
}

static void
callbackSpacingMode(int b) {
    g_spacingMode = (SpacingMode)b;
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

        case 'D': writeDigraph(g_plansTable->GetTopologyMap(),
            mods==GLFW_MOD_SHIFT ? -1 : g_currentPlanIndex);
            break;

        case 'V' : g_saveSVG = true; break;

        case '=':  {
            g_tessLevel+=5;
            if (g_tessLevel%2==0) ++g_tessLevel;
            rebuildMeshes();
        } break;
        case '-': {
            g_tessLevel-=5;
            if (g_tessLevel%2==0) ++g_tessLevel;
            g_tessLevel = std::max(g_tessLevelMin, g_tessLevel);
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

    g_hud.AddCheckBox("Dynamic Isolation (D)", g_useDynamicIsolation==1,
        10, 70, callbackCheckBox, kHUD_CB_USE_DYNAMIC_ISOLATION, 'd');


    g_hud.AddCheckBox("Smooth Corner Patch (O)", g_smoothCornerPatch!=0,
                      10, 90, callbackCheckBox, kHUD_CB_SMOOTH_CORNER_PATCH, 'o');

    g_hud.AddCheckBox("Single Crease Patch (S)", g_singleCreasePatch!=0,
                          10, 110, callbackCheckBox, kHUD_CB_SINGLE_CREASE_PATCH, 's');

    g_hud.AddCheckBox("Inf Sharp Patch (I)", g_infSharpPatch!=0,
                          10, 130, callbackCheckBox, kHUD_CB_INF_SHARP_PATCH, 'i');

    g_hud.AddCheckBox("Vert IDs", g_DrawVertIDs!=0,
        10, 160, callbackCheckBox, kHUD_CB_DISPLAY_VERT_IDS);

    g_hud.AddCheckBox("Face IDs", g_DrawFaceIDs!=0,
        10, 180, callbackCheckBox, kHUD_CB_DISPLAY_FACE_IDS);

    g_hud.AddCheckBox("Node IDs", g_DrawNodeIDs!=0,
        10, 200, callbackCheckBox, kHUD_CB_DISPLAY_NODE_IDS);

    g_hud.AddCheckBox("Tess Factors", g_DrawTessFactors!=0,
        10, 220, callbackCheckBox, kHUD_CB_DISPLAY_TESSFACTORS);

    int endcap_pulldown = g_hud.AddPullDown(
        "End cap (E)", 10, 250, 200, callbackEndCap, 'e');
    //g_hud.AddPullDownButton(endcap_pulldown, "None",
    //    Far::ENDCAP_NONE, g_endCap == Far::ENDCAP_NONE);
    g_hud.AddPullDownButton(endcap_pulldown, "Bilinear",
        Far::ENDCAP_BILINEAR_BASIS, g_endCap == Far::ENDCAP_BILINEAR_BASIS);
    g_hud.AddPullDownButton(endcap_pulldown, "BSpline",
        Far::ENDCAP_BSPLINE_BASIS, g_endCap == Far::ENDCAP_BSPLINE_BASIS);
    g_hud.AddPullDownButton(endcap_pulldown, "GregoryBasis",
        Far::ENDCAP_GREGORY_BASIS, g_endCap == Far::ENDCAP_GREGORY_BASIS);
    //g_hud.AddPullDownButton(endcap_pulldown, "LegacyGregory",
    //    Far::ENDCAP_LEGACY_GREGORY, g_endCap == Far::ENDCAP_LEGACY_GREGORY);


    g_hud.AddCheckBox("Dynamic Tess", g_dynamicTess!=0,
        10, 570, callbackCheckBox, kHUD_CB_DYNAMIC_TESS);


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
    //g_hud.AddPullDownButton(shading_pulldown, "Tess Factors",
    //    ::SHADING_TESS_FACTORS, g_shadingMode == ::SHADING_TESS_FACTORS);

    int spacing_pulldown = g_hud.AddPullDown("Spacing Mode (M)", 425, 10, 250, callbackSpacingMode, 'm');
    g_hud.AddPullDownButton(spacing_pulldown, "fractional odd", FRACTIONAL_ODD, g_spacingMode == FRACTIONAL_ODD);
    g_hud.AddPullDownButton(spacing_pulldown, "fractional even", FRACTIONAL_EVEN, g_spacingMode == FRACTIONAL_EVEN);
    g_hud.AddPullDownButton(spacing_pulldown, "equal", EQUAL, g_spacingMode == EQUAL);

    g_hud.AddSlider("Font Scale", 0.0f, 0.1f, 0.01f,
                    -800, -50, 100, false, callbackFontScale, 0);

    for (int i = 1; i < 11; ++i) {
        char level[16];
        sprintf(level, "Lv. %d", i);
        g_hud.AddRadioButton(3, level, i==g_level, 10, 310+i*20, callbackLevel, i, '0'+(i%10));
    }

    for (int i = 1; i < 11; ++i) {
        char level[16];
        sprintf(level, "Lv. %d", i);
        g_hud.AddRadioButton(4, level, i==g_dynamicLevel, 100, 310+i*20, callbackDynamicLevel, i, '0'+(i%10));
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

    static const char windowTitle[] = "OSD TopologyMap test harness";

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

