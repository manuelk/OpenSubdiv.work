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


#include "../osd/d3d11CharacteristicTable.h"

#include <D3D11.h>
#include "../far/characteristicMap.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Osd {


D3D11CharacteristicTable::D3D11CharacteristicTable() {
}

D3D11CharacteristicTable::~D3D11CharacteristicTable() {
    if (_characteristicTreesBuffer) _characteristicTreesBuffer->Release();
    if (_plansBuffer) _plansBuffer->Release();
}

D3D11CharacteristicTable *
D3D11CharacteristicTable::Create(Far::CharacteristicMap const & charmap,
    Far::PlanVector const & plans, ID3D11DeviceContext *pd3d11DeviceContext) {

    D3D11CharacteristicTable *instance = new D3D11CharacteristicTable();
    if (instance->allocate(charmap, plans, pd3d11DeviceContext))
        return instance;
    delete instance;
    return 0;
}

bool
D3D11CharacteristicTable::allocate(
    Far::CharacteristicMap const & charmap, Far::PlanVector const & plans,
        ID3D11DeviceContext *pd3d11DeviceContext) {

    ID3D11Device *pd3d11Device = NULL;
    pd3d11DeviceContext->GetDevice(&pd3d11Device);
    assert(pd3d11Device);

    {   // trees
        int size = charmap.GetCharacteristicTreeSizeTotal();

        // index buffer
        D3D11_BUFFER_DESC bd;
        ZeroMemory(&bd, sizeof(bd));
        bd.ByteWidth = size * sizeof(int);
        bd.Usage = D3D11_USAGE_DYNAMIC;
        bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
        bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        bd.MiscFlags = 0;
        bd.StructureByteStride = sizeof(int);
        HRESULT hr = pd3d11Device->CreateBuffer(&bd, NULL, &_characteristicTreesBuffer);
        if (FAILED(hr)) {
            return false;
        }

        D3D11_MAPPED_SUBRESOURCE mappedResource;
        hr = pd3d11DeviceContext->Map(
            _characteristicTreesBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
        if (FAILED(hr)) {
            return false;
        }
        int * data = (int *) mappedResource.pData;
        for (int i=0; i<charmap.GetNumCharacteristics(); ++i) {
            Far::Characteristic const * ch = charmap.GetCharacteristic(i);
            memcpy(data + ch->GetTreeOffset(), ch->GetTreeData(),ch->GetTreeSize() * sizeof(int));
        }
        pd3d11DeviceContext->Unmap(_characteristicTreesBuffer, 0);
    }


    return true;
}

}  // end namespace Osd

}  // end namespace OPENSUBDIV_VERSION
}  // end namespace OpenSubdiv
