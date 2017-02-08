#pragma once

#include "Action.h"
#include "ActionMemory.h"

class ActionRecorder {
public:
	ActionMemory memory;

	ActionRecorder() {}

	void recordHistory(const int& _moved, const float& _reward) {
		memory.append(_moved, _reward);
	}

	void replayHistory() {
		// TBD
		return;
	}

	void clearHistory() {
		memory.reset();
	}
};