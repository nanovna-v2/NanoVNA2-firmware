#include "uihw.hpp"
#include "debouncer.hpp"
#include <board.hpp>

namespace UIHW {
	small_function<void(UIEvent evt)> emitEvent;
	Debouncer touchDebouncer;

	void init(uint32_t buttonCheckIntervalMicros) {
		// touch
		touchDebouncer.pad = board::xpt2046_irq;
		touchDebouncer.transitionThreshold = 10000/buttonCheckIntervalMicros; // 10ms
		touchDebouncer.init();
	}

	void checkButtons() {
		UIEvent evt;
		if(touchDebouncer.checkChanged()) {
			evt.button = UIEventButtons::Touch;
			evt.type = touchDebouncer.state ? UIEventTypes::Up : UIEventTypes::Down;
			emitEvent(evt);
		}
	}

	void touchPosition(int& x, int& y) {
		uint16_t touchX, touchY;
		board::xpt2046.getRaw(touchX, touchY);
		x = touchX;
		y = touchY;
	}
}
