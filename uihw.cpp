#include "uihw.hpp"
#include "debouncer.hpp"
#include <board.hpp>

namespace UIHW {
	small_function<void(UIEvent evt)> emitEvent;
	Debouncer touchDebouncer;
	Debouncer buttonDebouncer[3];

	void init(uint32_t buttonCheckIntervalMicros) {
		// touch
		touchDebouncer.pad = board::xpt2046_irq;
		touchDebouncer.transitionThreshold = 10000/buttonCheckIntervalMicros; // 10ms
		touchDebouncer.init();

		buttonDebouncer[0].pad = board::LEVER_LEFT;
		buttonDebouncer[1].pad = board::LEVER_CENTER;
		buttonDebouncer[2].pad = board::LEVER_RIGHT;

		for(int i=0; i<3; i++) {
			if(board::LEVER_POLARITY)
				pinMode(buttonDebouncer[i].pad, INPUT_PULLDOWN);
			else pinMode(buttonDebouncer[i].pad, INPUT_PULLUP);

			buttonDebouncer[i].transitionThreshold = touchDebouncer.transitionThreshold;
			buttonDebouncer[i].init();
		}
	}

	void checkButtons() {
		UIEvent evt;
		if(touchDebouncer.checkChanged()) {
			evt.button = UIEventButtons::Touch;
			evt.type = touchDebouncer.state ? UIEventTypes::Up : UIEventTypes::Down;
			emitEvent(evt);
		}
		UIEventButtons buttons[3] = {UIEventButtons::LeverLeft, UIEventButtons::LeverCenter, UIEventButtons::LeverRight};
		for(int i=0; i<3; i++) {
			evt.button = buttons[i];
			if(buttonDebouncer[i].checkChanged()) {
				evt.type = (buttonDebouncer[i].state == board::LEVER_POLARITY) ? UIEventTypes::Down : UIEventTypes::Up;
				emitEvent(evt);
				if(evt.type == UIEventTypes::Up) {
					evt.type = UIEventTypes::Click;
					emitEvent(evt);
				}
			}
		}
	}

	void touchPosition(uint16_t& x, uint16_t& y) {
		if(!board::xpt2046.isTouching()) {
			x = y = (uint16_t) -1;
			return;
		}
		board::xpt2046.getRaw(x, y);
	}
}
