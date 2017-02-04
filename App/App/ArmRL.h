#pragma once

#include "CircularQueue.h"
#include "NeuralNetwork.h"
#include "AnimationMemory.h"

class ArmRL {
public:
	AnimationMemory memory_;

	ArmRL() {}

	void recordHistory(const int& _action_shoulder, const int& _action_elbow, const float& _distance, const float& _reward) {
		memory_.append(_action_shoulder, _action_elbow, _distance, _reward);
	}

	void clearHistory() {
		memory_.reset();
	}
};