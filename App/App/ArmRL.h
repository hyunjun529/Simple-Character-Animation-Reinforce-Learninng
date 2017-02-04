#pragma once

#include "CircularQueue.h"
#include "NeuralNetwork.h"
#include "AnimationMemory.h"

class ArmRL {
public:
	float gamma_;

	int num_input_histories_;
	int num_state_variables_;
	int num_game_actions_;

	NeuralNetwork nn_shoulder_;
	NeuralNetwork nn_elbow_;
	AnimationMemory memory_;

	VectorND<float> vector_shoulder_old_input_, vector_shoulder_next_input_, vector_shoulder_reward_;
	VectorND<float> vector_elbow_old_input_, vector_elbow_next_input_, vector_elbow_reward_;

	ArmRL()
		:gamma_(0.9f),
		num_input_histories_(1),
		num_state_variables_(1),
		num_game_actions_(3) {

		const int num_hidden_layers_ = 1;

		nn_shoulder_.initialize(num_state_variables_ * num_input_histories_, num_game_actions_, num_hidden_layers_);
		nn_elbow_.initialize(num_state_variables_ * num_input_histories_, num_game_actions_, num_hidden_layers_);

		for (int i = 0; i <= num_hidden_layers_ + 1; i++) {
			nn_shoulder_.layers_[i].act_type_ = LayerBase::ReLU;
			nn_elbow_.layers_[i].act_type_ = LayerBase::ReLU;
		}

		nn_shoulder_.eta_ = 1e-5;
		nn_elbow_.eta_ = 1e-5;

		nn_shoulder_.alpha_ = 0.9;
		nn_elbow_.alpha_ = 0.9;

		vector_shoulder_old_input_.initialize(nn_shoulder_.num_input_, true);
		vector_shoulder_next_input_.initialize(nn_shoulder_.num_input_, true);

		vector_elbow_old_input_.initialize(nn_elbow_.num_input_, true);
		vector_elbow_next_input_.initialize(nn_elbow_.num_input_, true);
	}

	void recordHistory(const int& _action_shoulder, const int& _action_elbow, const float& _distance, const float& _reward) {
		memory_.append(_action_shoulder, _action_elbow, _distance, _reward);
	}

	void recordVectorHistory(const int& _action_shoulder, const int& _action_elbow, const float& _distance, const float& _reward,
		const VectorND<float> _shoulder_state_vector_array,
		const VectorND<float> _elbow_state_vector_array,
		const VectorND<float> _shoulder_q_values_array,
		const VectorND<float> _elbow_q_values_array) {
		memory_.append(_action_shoulder, _action_elbow, _distance, _reward
			,_shoulder_state_vector_array, _elbow_state_vector_array,
			_shoulder_q_values_array, _elbow_q_values_array);
	}

	/********************************************************************************************
	* start IT IS TOO BAD : shoulder
	*********************************************************************************************/

	void clearHistory() {
		memory_.reset();
	}


	void trainReward() {
		trainReward(0);	// train with last memory
	}

	void trainRewardMemory() {
		for (int ix_from_end = 0; ix_from_end > -(memory_.num_elements - num_input_histories_); ix_from_end--)
			trainReward(ix_from_end);
	}

	void trainReward(const int ix_from_end) {

	}

	void forward() {

	}

	void makeInputVectorFromHistory(const int& ix_from_end, VectorND<float>& input) {

	}

	/********************************************************************************************
	* end IT IS TOO BAD : shoulder
	*********************************************************************************************/
};