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

#ifndef OPENSUBDIV3_FAR_NEIGHBORHOOD_BUILDER_H
#define OPENSUBDIV3_FAR_NEIGHBORHOOD_BUILDER_H

#include "../version.h"

#include "../far/neighborhood.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class TopologyLevel;

// Topoloogy neighborhood descriptor : uniquely identifies topological
// configurations of a mesh
//
class NeighborhoodBuilder {

public:

    /// \brief Constructor
    ///
    /// @param numNeighborhoods  The number of neighborhoods we expect
    ///                          to generate.
    ///
    /// @param maxValence        The maximum valence of a face in the mesh
    ///
    NeighborhoodBuilder(int numNeighborhoods, int maxValence=256);

    /// \brief Destructor
    ~NeighborhoodBuilder();

    /// Creates a topology neighborhood around a given patch
    ///
    /// @param level      The topology level containing the control face
    ///
    /// @param faceIndex  Index of the face in the topology level
    ///
    /// @param startEdge  Rotates the neighborhood around the face
    ///
    /// @param collect    Marks the neighborhood for garbage collection
    ///                   when this builder is deleted
    ///
    Neighborhood const * Create(TopologyLevel const & level,
        int faceIndex, int startEdge=0, bool collect=true);

private:

    void clear(bool clearVertRemaps=true);

    // Remaps vertex indices to an index-space local to the neighborhood.
    // It is assumed that patches with identical topological configurations
    // will have identical neighborhoods.
    int remapVert(int vert);

    // Adds a face and its vertices to a neighborhood
    void addFace(TopologyLevel const & level, int faceIndex, int origin);

    // Sharpness tags manipulation
    int findTag(float sharpness, int origin, int end);
    void addEdgeTag(TopologyLevel const & level, int edgeIndex, int vertIndex);
    void addVertTag(TopologyLevel const & level, int vertIndex);

private:

    std::vector<int> _faces,
                     _faceValences,
                     _faceVerts,
                     _vertRemaps;

    typedef Neighborhood::Tag Tag;
    std::vector<Tag> _tags;

    std::vector<Neighborhood *> _garbageCollector;
};


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_NEIGHBORHOOD_BUILDER_H */

