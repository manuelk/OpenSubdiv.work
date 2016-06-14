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

struct Vertex {
    vec4 position;
    vec4 normal;
    vec3 color;
};

//--------------------------------------------------------------
// Vertex Shader
//--------------------------------------------------------------
#ifdef VERTEX_SHADER

in vec3 my_position;
in vec3 my_normal;
in vec3 my_color;

out vec4 vPosition;
out vec4 vNormal;
out vec3 vColor;

void main() {
    vPosition = ModelViewMatrix * vec4(my_position.xyz, 1.0);
    vNormal = ModelViewMatrix * vec4(normalize(my_normal.xyz), 1.0);
    vColor = my_color;
    gl_Position = ProjectionMatrix * vPosition;
}
#endif

//--------------------------------------------------------------
// Fragment Shader
//--------------------------------------------------------------
#ifdef FRAGMENT_SHADER

in vec4 vPosition;
in vec4 vNormal;
in vec3 vColor;

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

vec4 lighting(vec4 diffuse, vec3 Peye, vec3 Neye) {

    vec4 color = vec4(0),
         ambientColor = vec4(0);

    for (int i = 0; i < NUM_LIGHTS; ++i) {

        vec4 Plight = lightSource[i].position;

        vec3 l = (Plight.w == 0.0)
                    ? normalize(Plight.xyz) : normalize(Plight.xyz - Peye);

        vec3 n = normalize(Neye);
        vec3 h = normalize(l + vec3(0,0,1));    // directional viewer

        float d = max(0.0, dot(n, l));
        float s = pow(max(0.0, dot(n, h)), 500.0f);

        color += lightSource[i].ambient * ambientColor
            + d * lightSource[i].diffuse * diffuse
            + s * lightSource[i].specular;
    }

    color.a = 1;
    return color;
}

out vec4 finalColor;

void main() {
    vec3 N = (gl_FrontFacing ? vNormal.xyz : -vNormal.xyz);

    vec4 color = vec4(vColor, 1.0);

    vec4 Cf = lighting(color, vPosition.xyz, N);

    //finalColor = Cf;
    finalColor = color + 0.00001 * vec4(Cf.xyz, 1.0);
    //finalColor = color;
}

#endif
