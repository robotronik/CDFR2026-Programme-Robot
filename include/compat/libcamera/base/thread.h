#pragma once

// Compatibility shim for libcamera's `Thread`, exposing only what the capture
// code needs to reach the event dispatcher. Debian/Ubuntu and Raspberry Pi OS
// ship libcamera-dev without this header (nor its `base/message.h` and
// `base/utils.h` dependencies), even though the class is part of the upstream
// public API.
//
// Only non-virtual, exported accessors are declared: they do not depend on the
// object layout, so this declaration stays ABI-compatible with libcamera 0.2/0.3
// as long as the code below never constructs a Thread or touches its vtable.

#include <sys/types.h>

namespace libcamera {

class EventDispatcher;

class Thread
{
public:
	static Thread *current();
	static pid_t currentId();

	EventDispatcher *eventDispatcher();
};

} /* namespace libcamera */
