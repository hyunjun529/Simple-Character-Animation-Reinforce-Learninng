#pragma once

#include "Action.h"
#include "VectorND.h"
#include <vector>

class AnimationMemory {
public:
	int num_elements = 0;
	int num_reserve = 10000;

	std::vector<float> distance_fist_to_head_array_;
	std::vector<float> reward_array_;

	std::vector<int> shoulder_action_array_; // it's selected
	std::vector<int> elbow_action_array_;

	std::vector<VectorND<float>> shoulder_state_vector_array_;
	std::vector<VectorND<float>> elbow_state_vector_array_;

	std::vector<VectorND<float>> shoulder_q_values_array_;
	std::vector<VectorND<float>> elbow_q_values_array_;

	AnimationMemory() {}

	void reset() {
		num_elements = 0;

		shoulder_action_array_.clear();
		elbow_action_array_.clear();
		distance_fist_to_head_array_.clear();
		reward_array_.clear();

		shoulder_state_vector_array_.clear();
		elbow_state_vector_array_.clear();
		shoulder_q_values_array_.clear();
		elbow_q_values_array_.clear();
	}

	// case about not use VectorND
	void append(const int& _action_shoulder, const int& _action_elbow, const float& _distance, const float& _reward) {
		num_elements++;

		shoulder_action_array_.push_back(_action_shoulder);
		elbow_action_array_.push_back(_action_elbow);
		distance_fist_to_head_array_.push_back(_distance);
		reward_array_.push_back(_reward);
	}

	// case about use VectorND
	void append(const int& _action_shoulder, const int& _action_elbow, const float& _distance, const float& _reward,
		const VectorND<float> _shoulder_state_vector_array,
		const VectorND<float> _elbow_state_vector_array,
		const VectorND<float> _shoulder_q_values_array,
		const VectorND<float> _elbow_q_values_array) {
		num_elements++;

		shoulder_action_array_.push_back(_action_shoulder);
		elbow_action_array_.push_back(_action_elbow);
		distance_fist_to_head_array_.push_back(_distance);
		reward_array_.push_back(_reward);

		shoulder_state_vector_array_.push_back(_shoulder_state_vector_array);
		elbow_state_vector_array_.push_back(_elbow_state_vector_array);
		shoulder_q_values_array_.push_back(_shoulder_q_values_array);
		elbow_q_values_array_.push_back(_elbow_q_values_array);
	}
	
	// get Reward
	const float& getRewardFromLast(const int& ix_from_last)
	{
		return reward_array_[num_elements - 1 + ix_from_last];
	}


	// about Shoulder
	const VectorND<float>& getShoulderStateVectorFromLast(const int& ix_from_last) // ix_from_last = 0 returns last element, use -1, -2 ,...
	{
		return shoulder_state_vector_array_[num_elements - 1 + ix_from_last];
	}

	const int& getShoulderSelectedIxFromLast(const int& ix_from_last)
	{
		return shoulder_action_array_[num_elements - 1 + ix_from_last];
	}

	const VectorND<float>& getShoulderQValuesFromLast(const int& ix_from_last) // ix_from_last = 0 returns last element, use -1, -2 ,...
	{
		return shoulder_q_values_array_[num_elements - 1 + ix_from_last];
	}

	// about Elbow
	const VectorND<float>& getElbowStateVectorFromLast(const int& ix_from_last) // ix_from_last = 0 returns last element, use -1, -2 ,...
	{
		return elbow_state_vector_array_[num_elements - 1 + ix_from_last];
	}

	const int& getElbowSelectedIxFromLast(const int& ix_from_last)
	{
		return elbow_action_array_[num_elements - 1 + ix_from_last];
	}

	const VectorND<float>& getElbowQValuesFromLast(const int& ix_from_last) // ix_from_last = 0 returns last element, use -1, -2 ,...
	{
		return elbow_q_values_array_[num_elements - 1 + ix_from_last];
	}
};