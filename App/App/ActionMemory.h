#pragma once

#include "VectorND.h"
#include <vector>

class ActionMemory {
public:
	int num_elements = 0;
	int num_reserve = 10000;

	std::vector<int> moved_array;
	std::vector<int> selected_array;
	std::vector<float> reward_array;

	ActionMemory() {}

	void reset() {
		num_elements = 0;

		moved_array.clear();
		selected_array.clear();
		reward_array.clear();
	}

	void append(const int& _moved, const int& _selected, const float& _reward) {
		num_elements++;

		moved_array.push_back(_moved);
		selected_array.push_back(_selected);
		reward_array.push_back(_reward);
	}
};