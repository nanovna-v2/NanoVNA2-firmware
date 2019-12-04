#pragma once
#include <mculib/small_function.hpp>
#include <stdint.h>

// hardware interfacing for the UI

namespace UIHW {
	enum class UIEventButtons: uint8_t {
		None = 0,
		Touch,
		LeverLeft, LeverRight, LeverCenter
	};
	enum class UIEventTypes: uint8_t {
		None = 0,
		Down,			// button down
		Up,				// button up
		Click,			// button down then up, with no ticks in between (if tick enabled for this button)
						// button down (if tick disabled for this button)
		DoubleClick,	// two clicks separated by a short interval; the Click event is suppressed in both cases
		LongPress,		// fires some time after button is held down
		Tick			// regular timed event when button is held down
	};
	struct UIEvent {
		UIEventButtons button;
		UIEventTypes type;

		// touchscreen was pressed
		bool isTouchPress() {
			return (button == UIEventButtons::Touch)
				&& (type == UIEventTypes::Down);
		}
		bool isTouchRelease() {
			return (button == UIEventButtons::Touch)
				&& (type == UIEventTypes::Up);
		}

		// lever was clicked
		bool isLeverClick() {
			return (button == UIEventButtons::LeverCenter)
				&& (type == UIEventTypes::Click);
		}
		bool isLeverLongPress() {
			return (button == UIEventButtons::LeverCenter)
				&& (type == UIEventTypes::LongPress);
		}
		// jog left or jog right
		bool isJog() {
			return (button == UIEventButtons::LeverLeft || button == UIEventButtons::LeverRight)
				&& (type == UIEventTypes::Down || type == UIEventTypes::Tick);
		}
		// jog return to center
		bool isJogEnd() {
			return (button == UIEventButtons::LeverLeft || button == UIEventButtons::LeverRight)
				&& (type == UIEventTypes::Up);
		}
		bool isJogLeft() {
			return button == UIEventButtons::LeverLeft &&
				(type == UIEventTypes::Down || type == UIEventTypes::Tick);
		}
		bool isJogRight() {
			return button == UIEventButtons::LeverRight &&
				(type == UIEventTypes::Down || type == UIEventTypes::Tick);
		}
		bool isTick() {
			return type == UIEventTypes::Tick;
		}
	};

	// hooks

	// called directly from interrupt context. When this is called, the
	// UI code should usually tell the application to call its
	// event handler from the main thread.
	extern small_function<void(UIEvent evt)> emitEvent;

	// buttonCheckIntervalMicros is how often (in microseconds)
	// the application will call checkButtons().
	void init(uint32_t buttonCheckIntervalMicros);

	// application should call this from a timer interrupt.
	void checkButtons();

	// get current touch position; sets x and y to -1 if there is no touch
	void touchPosition(uint16_t& x, uint16_t& y);
}
