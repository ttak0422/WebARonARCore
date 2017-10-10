// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "modules/vr/VRPlaneEvent.h"

#include "modules/vr/VRPlane.h"

namespace blink {

VRPlaneEvent::VRPlaneEvent() {}

VRPlaneEvent::VRPlaneEvent(const AtomicString& type,
                           bool canBubble,
                           bool cancelable,
                           VRDisplay* display,
                           HeapVector<Member<VRPlane>> planes)
    : Event(type, canBubble, cancelable),
      m_display(display),
      m_planes(planes) {}

VRPlaneEvent::VRPlaneEvent(const AtomicString& type,
                           const VRPlaneEventInit& initializer)
    : Event(type, initializer) {
  if (initializer.hasDisplay())
    m_display = initializer.display();

  if (initializer.hasPlanes())
    m_planes = initializer.planes();
}

VRPlaneEvent::~VRPlaneEvent() {}

const AtomicString& VRPlaneEvent::interfaceName() const {
  return EventNames::VRPlaneEvent;
}

DEFINE_TRACE(VRPlaneEvent) {
  visitor->trace(m_display);
  visitor->trace(m_planes);
  Event::trace(visitor);
}

}  // namespace blink
