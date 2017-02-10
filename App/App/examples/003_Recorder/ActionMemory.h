#pragma once

#include "Actions.h"
#include "VectorND.h"
#include <vector>

class ActionMemory {
public:
	int num_elements = 0;
	int num_reserve = 10000;

	std::vector<int> action_shoulder_array;
	std::vector<int> action_elbow_array;
	std::vector<VectorND<float>> state_vector_array_;
	std::vector<float> reward_array_;

	ActionMemory() {}

	void reset() {
		num_elements = 0;

		action_shoulder_array.clear();
		action_elbow_array.clear();
		state_vector_array_.clear();
		reward_array_.clear();
	}

	void append(const int& _action_shoulder, const int& _action_elbow, const VectorND<float>& _state, const float& _reward) {
		num_elements++;

		action_shoulder_array.push_back(_action_shoulder);
		action_elbow_array.push_back(_action_elbow);
		state_vector_array_.push_back(_state);
		reward_array_.push_back(_reward);
	}
};