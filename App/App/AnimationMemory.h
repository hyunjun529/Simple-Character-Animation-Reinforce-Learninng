#pragma once

#include "VectorND.h"
#include <vector>

class AnimationMemory {
public:
	int num_elements = 0;
	int num_reserve = 10000;

	std::vector<int> action_shoulder_array;
	std::vector<int> action_elbow_array;
	std::vector<float> distance_fist_to_head_array;
	std::vector<float> reward_array;

	AnimationMemory() {}

	void reset() {
		num_elements = 0;

		action_shoulder_array.clear();
		action_elbow_array.clear();
		distance_fist_to_head_array.clear();
		reward_array.clear();
	}

	void append(const int& _action_shoulder, const int& _action_elbow, const float& _distance, const float& _reward) {
		num_elements++;

		action_shoulder_array.push_back(_action_shoulder);
		action_elbow_array.push_back(_action_elbow);
		distance_fist_to_head_array.push_back(_distance);
		reward_array.push_back(_reward);
	}
};