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

#if defined(SHADING_VARYING_COLOR) || defined(SHADING_FACEVARYING_COLOR)
#undef OSD_USER_VARYING_DECLARE
#define OSD_USER_VARYING_DECLARE \
    vec3 color;

#undef OSD_USER_VARYING_ATTRIBUTE_DECLARE
#define OSD_USER_VARYING_ATTRIBUTE_DECLARE \
    layout(location = 1) in vec3 color;

#undef OSD_USER_VARYING_PER_VERTEX
#define OSD_USER_VARYING_PER_VERTEX() \
    outpt.color = color

#undef OSD_USER_VARYING_PER_CONTROL_POINT
#define OSD_USER_VARYING_PER_CONTROL_POINT(ID_OUT, ID_IN) \
    outpt[ID_OUT].color = inpt[ID_IN].color

#undef OSD_USER_VARYING_PER_EVAL_POINT
#define OSD_USER_VARYING_PER_EVAL_POINT(UV, a, b, c, d) \
    outpt.color = \
        mix(mix(inpt[a].color, inpt[b].color, UV.x), \
            mix(inpt[c].color, inpt[d].color, UV.x), UV.y)
#else
#define OSD_USER_VARYING_DECLARE
#define OSD_USER_VARYING_ATTRIBUTE_DECLARE
#define OSD_USER_VARYING_PER_VERTEX()
#define OSD_USER_VARYING_PER_CONTROL_POINT(ID_OUT, ID_IN)
#define OSD_USER_VARYING_PER_EVAL_POINT(UV, a, b, c, d)
#endif


//--------------------------------------------------------------
// Uniforms / Uniform Blocks
//--------------------------------------------------------------

layout(std140) uniform Transform {
    mat4 ModelViewMatrix;
    mat4 ProjectionMatrix;
    mat4 ModelViewProjectionMatrix;
    mat4 ModelViewInverseMatrix;
};

layout(std140) uniform Tessellation {
    float TessLevel;
};

//--------------------------------------------------------------
// Osd external functions
//--------------------------------------------------------------

mat4 OsdModelViewMatrix() {
    return ModelViewMatrix;
}
mat4 OsdProjectionMatrix() {
    return ProjectionMatrix;
}
mat4 OsdModelViewProjectionMatrix() {
    return ModelViewProjectionMatrix;
}
float OsdTessLevel() {
    return TessLevel;
}
int OsdPrimitiveIdBase() {
    // XXXX should be gl_primitiveID
    return 0;
}
//--------------------------------------------------------------
// Vertex Shader
//--------------------------------------------------------------
#ifdef VERTEX_SHADER

OSD_USER_VARYING_ATTRIBUTE_DECLARE

out block {
    vec3 position;
    OSD_USER_VARYING_DECLARE
} outpt;

void main() {
    outpt.position = ModelViewMatrix * position;
    OSD_USER_VARYING_PER_VERTEX();
}

#endif // VERTEX_SHADER

//----------------------------------------------------------
// Patches.TessControlBSpline
//----------------------------------------------------------
#ifdef TESS_CONTROL_SHADER

in block {
    OSD_USER_VARYING_DECLARE
} inpt[];

out block {
    OsdPerPatchVertexBezier v;
    OSD_USER_VARYING_DECLARE
} outpt[16];

layout(vertices = 16) out;

void main() {

    if (gl_InvocationID < 4) {
        gl_TessLevelOuter[gl_InvocationID] = 4;
    }
    if (gl_InvocationID < 4) {
        gl_TessLevelInner[gl_InvocationID] = 4;
    }
}

#endif // TESS_CONTROL_SHADER

//----------------------------------------------------------
// Patches.TessEvalBSpline
//----------------------------------------------------------
#ifdef TESS_EVAL_SHADER

layout(quads) in;
layout(OSD_SPACING) in;

patch in vec4 tessOuterLo, tessOuterHi;

in block {
    OsdPerPatchVertexBezier v;
    OSD_USER_VARYING_DECLARE
} inpt[];

out block {
    OutputVertex v;
    OSD_USER_VARYING_DECLARE
} outpt;

void main() {
}

#endif // TESS_EVAL_SHADER

//--------------------------------------------------------------
// Geometry Shader
//--------------------------------------------------------------
#ifdef GEOMETRY_SHADER

layout(triangles) in;

#define EDGE_VERTS 3

layout(triangle_strip, max_vertices = EDGE_VERTS) out;
in block {
    OutputVertex v;
    OSD_USER_VARYING_DECLARE
} inpt[EDGE_VERTS];

out block {
    OutputVertex v;
    OSD_USER_VARYING_DECLARE
} outpt;

void main() {

    gl_PrimitiveID = gl_PrimitiveIDIn;

    EndPrimitive();
}

#endif // GEOMETRY_SHADER

//--------------------------------------------------------------
// Fragment Shader
//--------------------------------------------------------------
#ifdef FRAGMENT_SHADER

in block {
    OutputVertex v;
    OSD_USER_VARYING_DECLARE
} inpt;

out vec4 outColor;

#define NUM_LIGHTS 2

struct LightSource {
    vec4 position;
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
};

layout(std140) uniform Lighting {
    LightSource lightSource[NUM_LIGHTS];
};

void main() {

    outColor = vec4(1, 1, 1, 1);
}

#endif // FRAGMENT_SHADER
