#include "uihw.hpp"
#include "debouncer.hpp"
#include <board.hpp>

namespace UIHW {
	small_function<void(UIEvent evt)> emitEvent;
	Debouncer touchDebouncer;
	Debouncer buttonDebouncer[3];
	int32_t tickCounter[3] = {};
	int32_t tickIntervalMicros = 100000, tickDelayMicros = 500000;
	int32_t tickInterval = 0, tickDelay = 0;

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
		tickInterval = tickIntervalMicros / buttonCheckIntervalMicros;
		tickDelay = tickDelayMicros / buttonCheckIntervalMicros;
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
				// only emit the click event if we haven't entered tick or long-press
				if(evt.type == UIEventTypes::Up && tickCounter[i] >= 0) {
					evt.type = UIEventTypes::Click;
					emitEvent(evt);
				}
			}
			if(buttonDebouncer[i].state == board::LEVER_POLARITY) {
				// button is depressed
				if(tickCounter[i] < 0) {
					// tick has started
					// ticks are only generated on lever left and right
					if(i == 0 || i == 2) {
						tickCounter[i]--;
						if(tickCounter[i] <= -tickInterval) {
							evt.type = UIEventTypes::Tick;
							emitEvent(evt);
							tickCounter[i] = -1;
						}
					}
					// lever center generates longpress events
					if(i == 1 && tickCounter[i] == -1) {
						evt.type = UIEventTypes::LongPress;
						emitEvent(evt);
						tickCounter[i] = -2;
					}
				} else {
					tickCounter[i]++;
					if(tickCounter[i] >= tickDelay)
						tickCounter[i] = -1;
				}
			} else {
				tickCounter[i] = 0;
			}
		}
	}

	bool touchPosition(uint16_t& x, uint16_t& y) {
		if(!board::xpt2046.isTouching())
			return false;
		board::xpt2046.getRaw(x, y);
		return board::xpt2046.isTouching();
	}
}
