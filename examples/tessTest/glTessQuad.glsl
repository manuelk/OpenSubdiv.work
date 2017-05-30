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



layout(std140) uniform Transform {
    mat4 ModelViewMatrix;
    mat4 ProjectionMatrix;
    mat4 ModelViewProjectionMatrix;
};


//--------------------------------------------------------------
// Vertex Shader
//--------------------------------------------------------------
#ifdef VERTEX_SHADER


in vec4 Position;
out vec3 vPosition;

void main() {
    vPosition = Position.xyz;
}
#endif



//--------------------------------------------------------------
// TESS CONTROL Shader
//--------------------------------------------------------------
#ifdef TESS_CONTROL_SHADER

layout(vertices = 4) out;
in vec3 vPosition[];
out vec3 tcPosition[];

uniform float TessLevelInner[2];
uniform float TessLevelOuter[4];

#define ID gl_InvocationID

void main() {
    tcPosition[ID] = vPosition[ID];

    if (ID == 0) {
        gl_TessLevelInner[0] = TessLevelInner[0];
        gl_TessLevelInner[1] = TessLevelInner[1];
        gl_TessLevelOuter[0] = TessLevelOuter[0];
        gl_TessLevelOuter[1] = TessLevelOuter[1];
        gl_TessLevelOuter[2] = TessLevelOuter[2];
        gl_TessLevelOuter[3] = TessLevelOuter[3];
    }
}
#endif

//--------------------------------------------------------------
// TESS EVAL Shader
//--------------------------------------------------------------
#ifdef TESS_EVAL_SHADER

layout(quads, SPACING_MODE) in;
in vec3 tcPosition[];
out vec3 tePosition;
out vec4 tePatchDistance;

void main() {

    float u = gl_TessCoord.x,
         v = gl_TessCoord.y;
    vec3 a = mix(tcPosition[0], tcPosition[1], u);
    vec3 b = mix(tcPosition[3], tcPosition[2], u);
    tePosition = mix(a, b, v);
    tePatchDistance = vec4(u, v, 1-u, 1-v);
    gl_Position = ProjectionMatrix * ModelViewMatrix * vec4(tePosition, 1);
}

#endif

//--------------------------------------------------------------
// Geometry Shader
//--------------------------------------------------------------
#ifdef GEOMETRY_SHADER

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;
in vec3 tePosition[3];
in vec4 tePatchDistance[3];
out vec3 gFacetNormal;
out vec4 gPatchDistance;
out vec3 gTriDistance;

void main() {

    vec3 A = tePosition[2] - tePosition[0];
    vec3 B = tePosition[1] - tePosition[0];
    gFacetNormal = (ModelViewMatrix * vec4(normalize(cross(A, B).xyz), 0.0)).xyz;
    
    gPatchDistance = tePatchDistance[0];
    gTriDistance = vec3(1, 0, 0);
    gl_Position = gl_in[0].gl_Position; EmitVertex();

    gPatchDistance = tePatchDistance[1];
    gTriDistance = vec3(0, 1, 0);
    gl_Position = gl_in[1].gl_Position; EmitVertex();

    gPatchDistance = tePatchDistance[2];
    gTriDistance = vec3(0, 0, 1);
    gl_Position = gl_in[2].gl_Position; EmitVertex();

    EndPrimitive();
}
#endif

//--------------------------------------------------------------
// Fragment Shader
//--------------------------------------------------------------
#ifdef FRAGMENT_SHADER

out vec4 FragColor;
in vec3 gFacetNormal;
in vec3 gTriDistance;
in vec3 gPatchDistance;
in float gPrimitive;

float amplify(float d, float scale, float offset) {
    d = scale * d + offset;
    d = clamp(d, 0, 1);
    d = 1 - exp2(-2*d*d);
    return d;
}

void main() {

    vec3 N = normalize(gFacetNormal);
    vec3 c = vec3(1.0, 0.5, 0.5);

    //float d1 = min(min(gTriDistance.x, gTriDistance.y), gTriDistance.z);
    //float d2 = min(min(gPatchDistance.x, gPatchDistance.y), gPatchDistance.z);
    //color = amplify(d1, 40, -0.5) * amplify(d2, 60, -0.5) * color;

    FragColor = vec4(c, 1.0);
}
#endif

