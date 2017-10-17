// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#ifndef VRPlaneEvent_h
#define VRPlaneEvent_h

#include "modules/EventModules.h"
#include "modules/vr/VRDisplay.h"
#include "modules/vr/VRPlaneEventInit.h"

namespace blink {

class VRPlane;

class VRPlaneEvent final : public Event {
  DEFINE_WRAPPERTYPEINFO();

 public:
  static VRPlaneEvent* create() { return new VRPlaneEvent; }
  static VRPlaneEvent* create(const AtomicString& type,
                              bool canBubble,
                              bool cancelable,
                              VRDisplay* display,
                              HeapVector<Member<VRPlane>> planes) {
    return new VRPlaneEvent(type, canBubble, cancelable, display, planes);
  }

  static VRPlaneEvent* create(const AtomicString& type,
                              const VRPlaneEventInit& initializer) {
    return new VRPlaneEvent(type, initializer);
  }

  ~VRPlaneEvent() override;

  VRDisplay* display() const { return m_display.get(); }
  HeapVector<Member<VRPlane>> planes() const { return m_planes; }

  const AtomicString& interfaceName() const override;

  DECLARE_VIRTUAL_TRACE();

 private:
  VRPlaneEvent();
  VRPlaneEvent(const AtomicString& type,
               bool canBubble,
               bool cancelable,
               VRDisplay*,
               HeapVector<Member<VRPlane>>);
  VRPlaneEvent(const AtomicString&, const VRPlaneEventInit&);

  Member<VRDisplay> m_display;
  HeapVector<Member<VRPlane>> m_planes;
};

}  // namespace blink

#endif  // VRPlaneEvent_h
