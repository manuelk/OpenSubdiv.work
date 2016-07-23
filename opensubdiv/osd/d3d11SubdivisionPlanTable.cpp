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

#include <D3D11.h>

#include "../osd/d3d11SubdivisionPlanTable.h"
#include "../far/characteristicMap.h"
#include "../far/error.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Osd {


D3D11SubdivisionPlanTable::D3D11SubdivisionPlanTable() :
    _numPlans(0), _characteristicTreesBuffer(0), _subdivisionPlansBuffer(0) {
}

D3D11SubdivisionPlanTable::~D3D11SubdivisionPlanTable() {
    if (_characteristicTreesBuffer) _characteristicTreesBuffer->Release();
    if (_subdivisionPlansBuffer) _subdivisionPlansBuffer->Release();
}

D3D11SubdivisionPlanTable *
D3D11SubdivisionPlanTable::Create(Far::SubdivisionPlanTable const & plansTable,
    ID3D11DeviceContext *pd3d11DeviceContext) {

    D3D11SubdivisionPlanTable *instance = new D3D11SubdivisionPlanTable();
    if (instance->allocate(plansTable, pd3d11DeviceContext))
        return instance;
    delete instance;
    return 0;
}

static ID3D11Buffer *
createBuffer(ID3D11Device * pd3d11Device, int numElements, int elementSize) {

    ID3D11Buffer *buffer = NULL;
    D3D11_BUFFER_DESC bd;
    bd.ByteWidth = numElements * elementSize;
    bd.Usage = D3D11_USAGE_IMMUTABLE;
    bd.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    bd.CPUAccessFlags = 0;
    bd.MiscFlags = 0;
    bd.StructureByteStride = elementSize;

    HRESULT hr = pd3d11Device->CreateBuffer(&bd, nullptr, &buffer);
    if (FAILED(hr)) {
        Far::Error(Far::FAR_RUNTIME_ERROR,
            "Error creating ID3D11Buffer (Osd::D3D11SubdivisionPlanTable)\n");
        return NULL;
    }
    return buffer;
}

void *
mapResource(ID3D11DeviceContext * pd3d11DeviceContext, ID3D11Buffer * buffer) {

    D3D11_MAPPED_SUBRESOURCE mappedResource;
    HRESULT hr = pd3d11DeviceContext->Map(buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
    if (FAILED(hr)) {
        Far::Error(Far::FAR_RUNTIME_ERROR,
            "Error mapping resource for ID3D11Buffer (Osd::D3D11SubdivisionPlanTable)\n");
        return false;
    }
    return mappedResource.pData;
}

bool
D3D11SubdivisionPlanTable::allocate(
    Far::SubdivisionPlanTable const & plansTable,
        ID3D11DeviceContext *pd3d11DeviceContext) {

    ID3D11Device *pd3d11Device = NULL;
    pd3d11DeviceContext->GetDevice(&pd3d11Device);
    assert(pd3d11Device);

    Far::CharacteristicMap const & charmap = plansTable.GetCharacteristicMap();
    Far::SubdivisionPlanVector const & plans = plansTable.GetSubdivisionPlans();

    {   // characteristic trees
        int size = charmap.GetCharacteristicTreeSizeTotal();

        ID3D11Buffer * buffer = createBuffer(pd3d11Device, size, sizeof(int));
        if (!buffer)
            return false;

        // concatenate the characteristic trees into a single array of ints
        int * data = (int *)mapResource(pd3d11DeviceContext, buffer);
        if (!data)
            return false;

        for (int i=0, offset=0; i<charmap.GetNumCharacteristics(); ++i) {
            Far::Characteristic const * ch = charmap.GetCharacteristic(i);
            memcpy(data + offset, ch->GetTreeData(), ch->GetTreeSize() * sizeof(int));
            offset += ch->GetTreeSize();
        }
        pd3d11DeviceContext->Unmap(buffer, 0);

        _characteristicTreesBuffer = buffer;
    }

    {   // subdivision plans

        Far::SubdivisionPlanVector const & plans = plansTable.GetSubdivisionPlans();

        ID3D11Buffer * buffer = createBuffer(
            pd3d11Device, (int)plans.size(), sizeof(Far::SubdivisionPlan));

        Far::SubdivisionPlan * data = (Far::SubdivisionPlan *)mapResource(pd3d11DeviceContext, buffer);
        if (!data)
            return false;

        memcpy(data, &plans[0], plans.size() * sizeof(Far::SubdivisionPlan));
        pd3d11DeviceContext->Unmap(buffer, 0);

        _subdivisionPlansBuffer = buffer;
        _numPlans = (int)plans.size();
    }

    return true;
}

}  // end namespace Osd

}  // end namespace OPENSUBDIV_VERSION
}  // end namespace OpenSubdiv
