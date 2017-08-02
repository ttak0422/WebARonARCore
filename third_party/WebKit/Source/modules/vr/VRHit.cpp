// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "modules/vr/VRHit.h"

namespace blink {

VRHit::VRHit()
{
    m_point = DOMFloat32Array::create(3);
    m_plane = DOMFloat32Array::create(4);
}

void VRHit::setHit(const device::mojom::blink::VRHitPtr& hitPtr)
{
    if (hitPtr.is_null())
        return;

    for (size_t i = 0; i < 3; i++) {
    	m_point->data()[i] = (float)hitPtr->point[i];
    }
    for (size_t i = 0; i < 4; i++) {
    	m_plane->data()[i] = (float)hitPtr->plane[i];
    }
}

DEFINE_TRACE(VRHit)
{
    visitor->trace(m_point);
    visitor->trace(m_plane);
}

} // namespace blink
