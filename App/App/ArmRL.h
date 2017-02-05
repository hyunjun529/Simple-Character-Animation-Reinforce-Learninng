#pragma once

#include "NeuralNetwork.h"
#include "AnimationMemory.h"

class ArmRL {
public:
	int num_input_histories_;
	int num_state_variables_;
	int num_game_actions_;

	float gamma_;

	NeuralNetwork nn_;
	AnimationMemory memory_;
	
	VectorND<float> old_input_vector_;
	VectorND<float> next_input_vector_;
	VectorND<float> reward_vector_;

	ArmRL()
		:gamma_(0.9f),
		num_input_histories_(1),
		num_state_variables_(1),
		num_game_actions_(3) {
	}

	/* push back this to history
	 * state = distance
	 * reward = 0.1, 0.0, 0.5
	 * choice = In, Out, Stay
	 * q_value = selected Q-value saved
	*/
	void recordHistory(const VectorND<float>& state_vector, const float& reward, const int& choice, const VectorND<float>& q_values)
	{
		memory_.append(state_vector, choice, reward, q_values);
	}
};