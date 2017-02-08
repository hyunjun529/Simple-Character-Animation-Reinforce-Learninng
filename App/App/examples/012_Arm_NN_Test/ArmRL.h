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
	AnimationMemory *memory_;

	VectorND<float> old_input_vector_;
	VectorND<float> next_input_vector_;
	VectorND<float> reward_vector_;

	ArmRL() {
		init();
	}

	void init() {
		gamma_ = 0.2f;
		num_input_histories_ = 1;
		num_state_variables_ = 1;
		num_game_actions_ = 3;

		//assert(num_exp_replay_ >= num_input_histories_);

		// initialize neural network
		const int num_hidden_layers = 1;

		nn_.initialize(num_state_variables_ * num_input_histories_, num_game_actions_, num_hidden_layers);

		for (int i = 0; i <= num_hidden_layers + 1; i++)
			nn_.layers_[i].act_type_ = LayerBase::ReLU;

		// h529 : Learning rate
		nn_.eta_ = 1e-5;
		// h529 : Momentum, 일종의 수치해석 기법
		nn_.alpha_ = 0.9;

		// h529 : Q-Learning 방정식에서 Gamma, 0~1사이로 학습 비율
		gamma_ = 0.9f;

		// initialize replay memory
		memory_ = new AnimationMemory();
		memory_->reserve(1e5);

		// h529 : value
		old_input_vector_.initialize(nn_.num_input_, true);
		next_input_vector_.initialize(nn_.num_input_, true);
	}

	void trainReward() {
		trainReward(0);
	}

	void trainRewardMemory() {
		for (int ix_from_end = 0; ix_from_end > -(memory_->num_elements_ - num_input_histories_); ix_from_end--)
			trainReward(ix_from_end);
	}

	void trainReward(const int ix_from_end) {
		// guess next Q value
		makeInputVectorFromHistory(ix_from_end, next_input_vector_);

		//const float reward_ix = history_.getValue(history_.array_.num_elements_ - 1 - 0 + ix_from_end).reward_;
		const float reward_ix = memory_->getRewardFromLast(ix_from_end);

		nn_.setInputVector(next_input_vector_);
		nn_.feedForward();

		const float next_Q = reward_ix < 0.0f ? 0.0f : nn_.getOutputValueMaxComponent();

		const int selected_dir = memory_->getSelectedIxFromLast(ix_from_end);

		//TODO: default training direction!

		makeInputVectorFromHistory(ix_from_end - 1, old_input_vector_);

		//const float high_reward_th = 0.8;
		const float high_reward_tr_ep = 0.1f;

		{
			int count = 0;
			while (true) {
				nn_.setInputVector(old_input_vector_); // old input
				nn_.feedForward();
				nn_.copyOutputVectorTo(false, reward_vector_);

				const float target = reward_ix + gamma_ * next_Q;
				const float error = ABS(reward_vector_[selected_dir] - target);

				reward_vector_[selected_dir] = reward_ix + gamma_ * next_Q;

				nn_.propBackward(reward_vector_);

				nn_.check();

				if (error < high_reward_tr_ep || count > 10000) {
					break;
				}

				count++;
			}
		}
	}

	void forward() {
		makeInputVectorFromHistory(0, old_input_vector_);
		nn_.setInputVector(old_input_vector_);
		nn_.feedForward();
	}

	void makeInputVectorFromHistory(const int& ix_from_end, VectorND<float>& input) {
		for (int r = 0, count = 0; r < num_input_histories_; r++, count += num_state_variables_) {
			const VectorND<float> &state_vector = memory_->getStateVectorFromLast(ix_from_end - r);
			input.copyPartial(state_vector, count, 0, num_state_variables_);
		}
	}

	/* push back this to history
	* state = distance
	* reward = 0.1, 0.0, 0.5
	* choice = In, Out, Stay
	* q_value = selected Q-value saved
	*/
	void recordHistory(const VectorND<float>& state_vector, const float& reward, const int& choice, const VectorND<float>& q_values)
	{
		memory_->append(state_vector, choice, reward, q_values);
	}
};