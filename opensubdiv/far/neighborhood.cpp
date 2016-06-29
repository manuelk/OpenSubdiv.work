//
//   Copyright 2015 Nvidia
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

#include "../far/neighborhood.h"
#include "../far/topologyRefiner.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

static void dumpArray(char const * name, ConstIndexArray const & a) {
    printf("%s = (%d) { ", name, a.size());
    for (int i=0; i<a.size(); ++i) {
        if (i>0) {
            printf(", ");
        }
        printf("%d", a[i]);
    }
    printf(" }\n");
}

void
Neighborhood::Print() const {
    printf("Neighborhood {\n");
    dumpArray("\tface valences", GetFaceValences());
    dumpArray("\tface verts", GetFaceVerts());
    ConstTagArray tags = GetTags();
    if (tags.size()>0) {
        //printf("\ttags = (%d) {\n", tags.size());
        for (int i=0; i<tags.size(); ++i) {
            Tag const & t = tags[i];
            printf("\ttag %d { origin=%d end=%d sharp=%f }\n", i, t.origin, t.end, t.sharpness);
        }
        //printf(" }\n");
    }
    dumpArray("\tvert remaps", GetVertRemaps());
    printf("\thash = 0x%X\n}\n", GetHash());
    fflush(stdout);
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
