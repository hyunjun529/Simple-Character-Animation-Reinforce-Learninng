#pragma once

#include "Action.h"
#include "ActionMemory.h"

class ActionRecorder {
public:
	ActionMemory memory;

	ActionRecorder() {}

	void recordHistory(const int& _moved, const int& _selected, const float& _reward) {
		memory.append(_moved, _selected, _reward);
	}

	void replayHistory() {
		// TBD
	}

	void clearHistory() {
		memory.reset();
	}
};