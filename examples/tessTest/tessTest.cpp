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
#include "./glTessQuad.h"
#include "./svg.h"

#include <string>
#include <fstream>
#include <sstream>
#include <thread>

#if _MSC_VER
    #define snprintf _snprintf
#endif



using namespace OpenSubdiv;

GLTessQuad::SpacingMode g_spacingMode = GLTessQuad::FRACTIONAL_EVEN;

float g_tessFactors[6] = { 2.5f, 2.5f, 2.5f, 2.5f, 2.5f, 2.5f };

int g_frame = 0,
    g_repeatCount = 0;

bool g_saveSVG = false;


// GUI variables
int   g_fullscreen = 0,
      g_mbutton[3] = {0, 0, 0},
      g_running = 1;

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


struct Point {

    Point() { }

    Point(float x, float y, float z) { xyz[0]=x; xyz[1]=y; xyz[2]=z; }

    void Clear( void * =0 ) {
         xyz[0] = xyz[1] = xyz[2] = 0.0f;
    }

    void AddWithWeight(Point const & src, float weight) {
        xyz[0] += weight * src.xyz[0];
        xyz[1] += weight * src.xyz[1];
        xyz[2] += weight * src.xyz[2];
    }

    float xyz[3];
};

//------------------------------------------------------------------------------

GLFont * g_font=0;

//------------------------------------------------------------------------------

GLMesh * g_tessMesh = 0;

//------------------------------------------------------------------------------

inline int computeNumEdgeVerts(float tessFactor) {
    return std::max(2, (int)std::ceil(tessFactor));
}

//------------------------------------------------------------------------------

GLTessQuad * g_tessQuad = 0;

static void
createMesh() {

    if (1) {
        delete g_tessQuad;
        g_tessQuad = new GLTessQuad;
        g_tessQuad->Init();
        g_tessQuad->SetSpacingMode(g_spacingMode);
        g_tessQuad->SetTessFactors(g_tessFactors+4, g_tessFactors);
    } else {

/*        
        std::vector<float> positions(numTris * 3 * 3),
                           normals(numTris * 3 * 3),
                           colors(numTris * 3 * 3),

        struct GLMesh::Topology topo;
        topo.positions = &positions[0];
        topo.normals   = &normals[0];
        topo.colors    = &colors[0];
        topo.nverts    = (int)positions.size()/3;
*/

        GLMesh::Topology topo = GLMesh::Topology::Cube();

        delete g_tessMesh;
        g_tessMesh = new GLMesh(topo, GLMesh::DRAW_WIREFRAME);
    }
}

//------------------------------------------------------------------------------
static void
rebuildMeshes() {

    if (! g_font) {
        g_font = new GLFont(g_hud.GetFontTexture());
    }
    g_font->Clear();

    createMesh();
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

if (g_saveSVG) {
    svg->SetLineWidth(0.1f);
    svg->SetLineColor(0.5f, 0.5f, 0.5f, 0.5f);
    svg->Write();
}

    // draw the mesh
    if (g_tessMesh) {
        g_tessMesh->Draw(g_transformUB, g_lightingUB, true);
    }

    if (g_tessQuad) {
        g_tessQuad->Draw(g_transformUB, true);
    }

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


        static char const * schemeNames[3] = { "BILINEAR", "CATMARK", "LOOP" };

        g_hud.DrawString(10, -40,  "Triangles  : %d", 0);
        g_hud.DrawString(10, -20,  "FPS        : %3.1f", fps);

        g_hud.Flush();
    }
    glFinish();

    //checkGLErrors("display leave");
}

//------------------------------------------------------------------------------
static void
callbackFontScale(float value, int) {
    g_font->SetFontScale(value);
}

static void
callbackTessFactors(float value, int data) {
    g_tessFactors[data]=value;

    rebuildMeshes();
}

static void
callbackSpacingMode(int b) {
    g_spacingMode = (GLTessQuad::SpacingMode)b;
    if (g_tessQuad) {
        g_tessQuad->SetSpacingMode(g_spacingMode);
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

        case 'V' : g_saveSVG = true; break;

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


    g_hud.AddSlider("Font Scale", 0.0f, 0.1f, 0.01f,
                    -800, -50, 100, false, callbackFontScale, 0);


    g_hud.AddSlider("Outer 0", 0.0f, 15.0f, g_tessFactors[0],
                    10, 10, 30, false, callbackTessFactors, 0);

    g_hud.AddSlider("Outer 1", 0.0f, 15.0f, g_tessFactors[1],
                    10, 50, 30, false, callbackTessFactors, 1);

    g_hud.AddSlider("Outer 2", 0.0f, 15.0f, g_tessFactors[2],
                    10, 90, 30, false, callbackTessFactors, 2);

    g_hud.AddSlider("Outer 3", 0.0f, 15.0f, g_tessFactors[3],
                    10, 130, 30, false, callbackTessFactors, 3);


    g_hud.AddSlider("Inner 0", 0.0f, 15.0f, g_tessFactors[4],
                    10, 200, 30, false, callbackTessFactors, 4);

    g_hud.AddSlider("Inner 1", 0.0f, 15.0f, g_tessFactors[5],
                    10, 240, 30, false, callbackTessFactors, 5);


    int spacing_pulldown = g_hud.AddPullDown("Spacing Mode (M)", 300, 10, 250,
                                                  callbackSpacingMode, 'm');
    g_hud.AddPullDownButton(spacing_pulldown, "fractional odd", GLTessQuad::FRACTIONAL_ODD,
                            g_spacingMode == GLTessQuad::FRACTIONAL_ODD);
    g_hud.AddPullDownButton(spacing_pulldown, "fractional even", GLTessQuad::FRACTIONAL_EVEN,
                            g_spacingMode == GLTessQuad::FRACTIONAL_EVEN);
    g_hud.AddPullDownButton(spacing_pulldown, "equal", GLTessQuad::EQUAL,
                            g_spacingMode == GLTessQuad::EQUAL);

    g_hud.Rebuild(windowWidth, windowHeight, frameBufferWidth, frameBufferHeight);
}

//------------------------------------------------------------------------------
static void
initGL() {
    glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
//#define CULLING
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

    GLUtils::PrintGLVersion();

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

