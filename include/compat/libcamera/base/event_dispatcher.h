#pragma once

// Compatibility shim for libcamera's `EventDispatcher` interface. Debian/Ubuntu
// and Raspberry Pi OS ship libcamera-dev without this header even though the
// class is part of the upstream public API, so the capture code provides the
// declaration it needs.
//
// `EventDispatcher` is a pure abstract interface: only virtual methods, no data
// members. Redeclaring it identically is ABI-compatible, provided the virtual
// member order matches upstream libcamera 0.2/0.3 exactly.

namespace libcamera {

class EventNotifier;
class Timer;

class EventDispatcher
{
public:
	virtual ~EventDispatcher();

	virtual void registerEventNotifier(EventNotifier *notifier) = 0;
	virtual void unregisterEventNotifier(EventNotifier *notifier) = 0;

	virtual void registerTimer(Timer *timer) = 0;
	virtual void unregisterTimer(Timer *timer) = 0;

	virtual void processEvents() = 0;

	virtual void interrupt() = 0;
};

} /* namespace libcamera */
