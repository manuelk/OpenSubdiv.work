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

#ifndef OPENSUBDIV3_FAR_NEIGHBORHOOD_H
#define OPENSUBDIV3_FAR_NEIGHBORHOOD_H

#include "../version.h"

#include "../far/types.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {


// Topoloogy neighborhood descriptor : uniquely identifies topological
// configurations of a mesh
//
class Neighborhood {

public:

    /// \brief Returns memory use in bytes (not including vertex remaps)
    int GetSize() const {
        return getSize(_faceValencesCount, _faceVertsCount, _tagsCount);
    }

    /// \brief Returns memory use in bytes (including vertex remaps)
    int GetSizeWithRemap() const {
        return getSize(_faceValencesCount, _faceVertsCount, _tagsCount, _vertRemapsCount);
    }

    /// \brief Returns the number of faces in the neighborhood
    int GetNumFaces() const {
        return _faceValencesCount;
    }

    /// \brief Returns the number of vertices in the neighborhood
    int GetNumVertices() const {
        return _vertRemapsCount;
    }

    /// \brief Returns the array of face valences
    ConstIndexArray GetFaceValences() const {
        return ConstIndexArray(getFaceValences(), _faceValencesCount);
    }

    /// \brief Returns the array of face-vert indices
    ConstIndexArray GetFaceVerts() const {
        return ConstIndexArray(getFaceVerts(), _faceVertsCount);
    }

    /// \brief Returns the array of vert indices remaps
    ConstIndexArray GetVertRemaps() const {
        return ConstIndexArray(getVertRemaps(), _vertRemapsCount);
    }

    /// \brief Returns the valence of the center face
    int GetValence() const {
        return getFaceValences()[0];
    }

    /// \brief Returns a hash of the neighborhood data (not including vert remaps)
    int GetHash() const {
        return hashBytes(this, GetSize());
    }

    /// \brief Returns true if both neighborhoods are topologically equivalent
    bool IsEquivalent(Neighborhood const & other) const;

public:

    //
    // Sharpness tags
    //
    struct Tag {

        Tag() { }
        Tag(int iorigin, int iend, float isharpness) :
            origin(iorigin), end(iend), sharpness(isharpness) { }

        int origin,
            end;
        float sharpness;
    };

    typedef Vtr::ConstArray<Tag> ConstTagArray;

    // Returns the array of tags (both edge & corners)
    ConstTagArray GetTags() const { return ConstTagArray(getTags(), _tagsCount); }

public:

    // Debug printout
    void Print() const;

private:

    friend class NeighborhoodBuilder;

    static int getSize(int valences, int faceVerts, int tags, int vertRemaps=0);

    int * getFaceValences() const { return (int *)(this+1); }

    int * getFaceVerts() const { return getFaceValences() + _faceValencesCount; }

    Tag * getTags() const { return (Tag *)(getFaceVerts()+_faceVertsCount); }

    int * getVertRemaps() const { return (int *)(getTags()+_tagsCount); }

    static int hashBytes(void const * bytes, size_t size);

private:

    // note : the counters are followed in memory with the actual data :
    // int  faceValences[faceValencesCount]
    // int  faceVerts[faceVertsCounts]
    // Tag  tags[tagsCount]
    // int  vertRemaps[vertRemapsCount]

    int _faceValencesCount,
        _faceVertsCount,
        _tagsCount,
        _vertRemapsCount;
};

inline bool
Neighborhood::IsEquivalent(Neighborhood const & other) const {
    int asize = GetSize(),
        bsize = other.GetSize();
    if (asize!=bsize) {
        return false;
    }
    return memcmp(this, &other, asize)==0;
}

inline int
Neighborhood::getSize(int valences, int faceVerts, int tags, int vertRemaps) {
    int size = sizeof(Neighborhood);
    size += (valences + faceVerts + vertRemaps)*sizeof(int);
    size += tags * sizeof(Tag);
    return size;
}

inline int
Neighborhood::hashBytes(void const * bytes, size_t size) {

    // FNV-1 hash (would FNV1-a be better ?)
    int hvalue = 0;
    unsigned char const * start = (unsigned char const *)bytes,
                        * end = start + size;
    while (start<end) {
        hvalue *= (unsigned int)0x01000193;
        hvalue ^= (unsigned int)*start;
        ++start;
    }
    return hvalue;
}


} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_NEIGHBORHOOD_H */

