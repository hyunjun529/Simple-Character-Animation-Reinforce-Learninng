#pragma once

#include "Action.h"
#include "AnimationMemory.h"

class AnimationRecorder {
public:
	AnimationMemory memory;

	AnimationRecorder() {}

	void recordHistory(const int& _action_shoulder, const int& _action_elbow, const float& _distance, const float& _reward) {
		memory.append(_action_shoulder, _action_elbow, _distance, _reward);
	}

	void replayHistory() {
		// TBD
		return;
	}

	void clearHistory() {
		memory.reset();
	}
};