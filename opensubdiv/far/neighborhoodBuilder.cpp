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

#include "../far/neighborhoodBuilder.h"
#include "../far/topologyLevel.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

NeighborhoodBuilder::NeighborhoodBuilder(int numPlans, int maxValence) {
    _faces.reserve(maxValence);
    _faceValences.reserve(maxValence);
    _faceVerts.reserve(maxValence);
    _tags.reserve(maxValence);
    _vertRemaps.reserve(maxValence);

    _garbageCollector.reserve(numPlans);
}

NeighborhoodBuilder::~NeighborhoodBuilder() {
    // delete temporary neighborhoods after support stencils
    // have been finalized
    for (int i=0; i<(int)_garbageCollector.size(); ++i) {
        free(_garbageCollector[i]);
    }
}

Neighborhood const *
NeighborhoodBuilder::Create(
    TopologyLevel const & level, int faceIndex, int startEdge, bool collect) {

    clear();

    ConstIndexArray fverts = level.GetFaceVertices(faceIndex),
                    fedges = level.GetFaceEdges(faceIndex);
    assert(fverts.size()==fedges.size());

    int origin = fverts[startEdge];

    // add 0-ring face to neighborhood
    addFace(level, faceIndex, origin);

    for (int vert=0; vert<fverts.size(); ++vert) {

        int index = (vert+startEdge)%fverts.size(),
            vertIndex = fverts[index],
            edgeIndex = fedges[index];

        // add 0-ring sharpness tags (if any)
        addVertTag(level, vertIndex);
        addEdgeTag(level, edgeIndex, vertIndex);

        // add 1-ring faces & vertices around vert 'vertIndex'.
        {
            ConstIndexArray faces = level.GetVertexFaces(vertIndex);
            // orient the enumeration starting from the 0-ring face
            int firstFace = faces.FindIndex(faceIndex);
            assert(firstFace!=INDEX_INVALID);
            for (int face=1; face<faces.size(); ++face) {
                addFace(level, faces[(firstFace+face)%faces.size()], vertIndex);
            }
         }

        // add 1-ring edge tags around vert 'vertIndex'
        {
            ConstIndexArray edges = level.GetVertexEdges(vertIndex);
            // orient the enumeration starting from the 0-ring face
            int firstEdge = edges.FindIndex(edgeIndex);
            assert(firstEdge!=INDEX_INVALID);
            for (int edge=1; edge<edges.size(); ++edge) {
                addEdgeTag(level, edges[(firstEdge+edge)%edges.size()], vertIndex);
            }
        }
    }

    // allocate & populate the neighborhood
    int vcount = (int)_faceValences.size(),
        fvcount = (int)_faceVerts.size(),
        tcount = (int)_tags.size(),
        rcount = (int)_vertRemaps.size(),
        size = Neighborhood::getSize(vcount, fvcount, tcount, rcount);

    Neighborhood * n = (Neighborhood *)malloc(size);

    n->_faceValencesCount = vcount;
    n->_faceVertsCount = fvcount;
    n->_tagsCount = tcount;
    n->_vertRemapsCount = rcount;

    memcpy(n->getFaceValences(), &_faceValences[0], vcount*sizeof(int));
    memcpy(n->getFaceVerts(), &_faceVerts[0], fvcount*sizeof(int));
    if (tcount>0) {
        memcpy(n->getTags(), &_tags[0], tcount*sizeof(Tag));
    }
    memcpy(n->getVertRemaps(), &_vertRemaps[0], rcount*sizeof(int));

    if (collect) {
        _garbageCollector.push_back(n);
    }

    return n;
}

//
// Helpers
//

void
NeighborhoodBuilder::clear(bool clearVertRemaps) {
    _faces.clear();
    _faceValences.clear();
    _faceVerts.clear();
    _tags.clear();
    if (clearVertRemaps) {
        _vertRemaps.clear();
    }
}

//
// Topology helpers
//

// Remaps vertex indices to an index-space local to the neighborhood.
// It is assumed that patches with identical topological configurations
// will have identical neighborhoods.
int
NeighborhoodBuilder::remapVert(int vert) {
    for (int i=0; i<(int)_vertRemaps.size(); ++i) {
        if (_vertRemaps[i] == vert)
            return i;
    }
    _vertRemaps.push_back(vert);
    return (int)_vertRemaps.size()-1;
}

// Adds a face and its vertices to a neighborhood
void
NeighborhoodBuilder::addFace(TopologyLevel const & level, int faceIndex, int origin) {

    // skip faces that we have already indexed
    for (int i=0; i<(int)_faces.size(); ++i) {
        if (_faces[i]==faceIndex) {
            return;
        }
    }

    ConstIndexArray fverts = level.GetFaceVertices(faceIndex);
    int valence = fverts.size(),
        startEdge = fverts.FindIndex(origin);
    assert(startEdge!=INDEX_INVALID);

    _faceValences.push_back(valence);

    for (int vert=0; vert<valence; ++vert) {
        int vertIndex = fverts[(vert+startEdge)%valence];
        _faceVerts.push_back(remapVert(vertIndex));
    }
    _faces.push_back(faceIndex);
}

//
// Sharpness tags helpers
//

int
NeighborhoodBuilder::findTag(float sharpness, int origin, int end) {
    for (int i=0; i<(int)_tags.size(); ++i) {
        if (_tags[i].origin==origin and _tags[i].end==end) {
            assert(_tags[i].sharpness == sharpness);
            return i;
        }
    }
    return INDEX_INVALID;
}

void
NeighborhoodBuilder::addEdgeTag(
    TopologyLevel const & level, int edgeIndex, int vertIndex) {

    float sharpness = level.GetEdgeSharpness(edgeIndex);
    if (sharpness>0.0f) {
        ConstIndexArray everts = level.GetEdgeVertices(edgeIndex);
#define SORT_BY_INDEX
#if defined(SORT_BY_INDEX)
        int origin = remapVert(everts[0]),        
            end = remapVert(everts[1]);
        if (origin>end) {
            std::swap(origin, end);
        }
#elif defined(SORT_BY_ORIGIN)
        // XXXX seems dangerous - we should probably just swap lowest index first
        int origin, end;
        if (vertIndex==everts[0]) {
            origin = remapVert(everts[0]);
            end = remapVert(everts[1]);
        } else {
            assert(vertIndex==everts[1]);
            origin = remapVert(everts[1]);
            end = remapVert(everts[0]);
        }
#else
        int origin = remapVert(everts[0]),        
            end = remapVert(everts[1]);
#endif        
        assert(origin!=INDEX_INVALID and end!=INDEX_INVALID);
        if (findTag(sharpness, origin, end)==INDEX_INVALID) {
            _tags.push_back(Tag(origin, end, sharpness));
        }
    }
}

void
NeighborhoodBuilder::addVertTag(TopologyLevel const & level, int vertIndex) {
    float sharpness = level.GetVertexSharpness(vertIndex);
    if (sharpness>0.0f) {
        int origin = remapVert(vertIndex),
            end = INDEX_INVALID;
        if (findTag(sharpness, origin, end)==INDEX_INVALID) {
            _tags.push_back(Tag(origin, end, sharpness));
        }
    }
}


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
