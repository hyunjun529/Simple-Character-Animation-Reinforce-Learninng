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

		memory_.num_elements = 0;
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

	void clearHistory() {
		memory_.reset();
	}

	/********************************************************************************************
	* start IT IS TOO BAD : shoulder
	*********************************************************************************************/

	void trainReward_shoulder_() {
		trainReward_shoulder_(0);	// train with last memory
	}

	void trainRewardMemory_shoulder_() {
		for (int ix_from_end = 0; ix_from_end > -(memory_.num_elements - num_input_histories_); ix_from_end--)
			trainReward_shoulder_(ix_from_end);
	}

	void trainReward_shoulder_(const int ix_from_end) {
		// guess next Q value
		makeInputVectorFromHistory_shoulder_(ix_from_end, vector_shoulder_next_input_);

		const float reward_ix = memory_.getRewardFromLast(ix_from_end);

		nn_shoulder_.setInputVector(vector_shoulder_next_input_);
		nn_shoulder_.feedForward();

		const float next_Q = reward_ix < 0.0f ? 0.0f : nn_shoulder_.getOutputValueMaxComponent();

		const int selected_dir = memory_.getSelectedIxFromLast_shoulder_(ix_from_end);
		//TODO: default training direction!

		makeInputVectorFromHistory_shoulder_(ix_from_end - 1, vector_shoulder_old_input_);

		//const float high_reward_th = 0.8;
		const float high_reward_tr_ep = 0.1f;

		{
			int count = 0;
			while (true)
			{
				nn_shoulder_.setInputVector(vector_shoulder_old_input_); // old input
				nn_shoulder_.feedForward();
				nn_shoulder_.copyOutputVectorTo(false, vector_shoulder_reward_);

				const float target = reward_ix + gamma_ * next_Q;
				const float error = ABS(vector_shoulder_reward_[selected_dir] - target);

				vector_shoulder_reward_[selected_dir] = reward_ix + gamma_ * next_Q;

				nn_shoulder_.propBackward(vector_shoulder_reward_);

				nn_shoulder_.check();

				if (error < high_reward_tr_ep || count > 10000) break;

				count++;
			}
		}
	}

	void forward_shoulder_() {
		makeInputVectorFromHistory_shoulder_(0, vector_shoulder_old_input_);
		nn_shoulder_.setInputVector(vector_shoulder_old_input_);
		nn_shoulder_.feedForward();
	}

	void makeInputVectorFromHistory_shoulder_(const int& ix_from_end, VectorND<float>& input) {
		for (int r = 0, count = 0; r < num_input_histories_; r++, count += num_state_variables_)
		{
			const VectorND<float> &state_vector =
				memory_.getStateVectorFromLast_shoulder_(ix_from_end - r);

			input.copyPartial(state_vector, count, 0, num_state_variables_);
		}
	}

	/********************************************************************************************
	* end IT IS TOO BAD : shoulder
	*********************************************************************************************/

	/********************************************************************************************
	* start IT IS TOO BAD : elbow
	*********************************************************************************************/

	void trainReward_elbow_() {
		trainReward_elbow_(0);	// train with last memory
	}

	void trainRewardMemory_elbow_() {
		for (int ix_from_end = 0; ix_from_end > -(memory_.num_elements - num_input_histories_); ix_from_end--)
			trainReward_elbow_(ix_from_end);
	}

	void trainReward_elbow_(const int ix_from_end) {

	}

	void forward_elbow_() {

	}

	void makeInputVectorFromHistory_elbow_(const int& ix_from_end, VectorND<float>& input) {

	}

	/********************************************************************************************
	* end IT IS TOO BAD : elbow
	*********************************************************************************************/
};