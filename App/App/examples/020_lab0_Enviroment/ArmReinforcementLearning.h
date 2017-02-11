#pragma once

#include "NeuralNetwork.h"
#include "ActionMemory.h"

class ArmReinforcementLearning
{
public:
	int num_exp_replay_;		// histories for experience replay training
	int num_input_histories_;	// input to nn
	int num_state_variables_;   // dimension of the state variables of the game
	int num_game_actions_;			// num outputs of the game

	float gamma_;

	NeuralNetwork nn_;
	ArmMemory memory_;

	VectorND<float> old_input_vector_, reward_vector_, next_input_vector_;

	void initialize()
	{
		//assert(num_exp_replay_ >= num_input_histories_);

		// initialize neural network
		const int num_hidden_layers = 1;

		// h529 : state_variables_ by input_histories로 2차원 배열 생성 > 이게 나중에 value로 넘어감
		nn_.initialize(num_state_variables_ * num_input_histories_, num_game_actions_, num_hidden_layers);

		for (int i = 0; i <= num_hidden_layers + 1; i++)
			nn_.layers_[i].act_type_ = LayerBase::ReLU;

		// h529 : Learning rate
		nn_.eta_ = 1e-5;
		// h529 : Momentum, 일종의 수치해석 기법
		nn_.alpha_ = 0.9f;

		// h529 : Q-Learning 방정식에서 Gamma, 0~1사이로 학습 비율
		gamma_ = 0.5f;

		// initialize replay memory
		memory_.reserve(1e5);

		// h529 : value
		old_input_vector_.initialize(nn_.num_input_, true);
		next_input_vector_.initialize(nn_.num_input_, true);

		//TODO: don't train if histories are not enough
	}

	void trainReward()
	{
		trainReward(0);	// train with last memory
						// train precious experience only
						//if (history_.getValue(-2).reward_ > 0.5)
						//	for (int i = 0; i < 10; i++)
						//		trainReward(0);
						//else
						//	trainReward(0);
	}

	void trainRewardMemory()
	{
		for (int ix_from_end = 0; ix_from_end > -(memory_.num_elements_ - num_input_histories_); ix_from_end--)
			trainReward(ix_from_end);
	}

	void trainReward(const int ix_from_end)
	{
		// guess next Q value
		makeInputVectorFromHistory(ix_from_end, next_input_vector_);

		const float reward_ix = memory_.getRewardFromLast(ix_from_end);

		nn_.setInputVector(next_input_vector_);
		nn_.feedForward();

		const float next_Q = reward_ix < 0.0f ? 0.0f : nn_.getOutputValueMaxComponent(); // final status test
																						 //const float next_Q = nn_.getOutputValueMaxComponent();

																						 //const int selected_dir = history_.getValue(history_.getLastIndex() -1 + ix_from_end).choice_; // last history is in one step future 
		const int selected_dir = memory_.getSelectedIxFromLast(ix_from_end);
		//TODO: default training direction!

		makeInputVectorFromHistory(ix_from_end - 1, old_input_vector_);

		//const float high_reward_th = 0.8;
		const float high_reward_tr_ep = 0.1f;

		//if (high_reward_th <= reward_ix || reward_ix < 0.1)
		{
			//std::cout << "High reward training start" << std::endl;

			int count = 0;
			while (true)
			// for(int r = 0; r < 1; r ++)
			{
				nn_.setInputVector(old_input_vector_); // old input
				nn_.feedForward();
				nn_.copyOutputVectorTo(false, reward_vector_);

				const float target = reward_ix + gamma_ * next_Q;
				const float error = ABS(reward_vector_[selected_dir] - target);

				reward_vector_[selected_dir] = reward_ix + gamma_ * next_Q;

				nn_.propBackward(reward_vector_);

				nn_.check();

				if (error < high_reward_tr_ep || count > 10000)
				{
					break;
				}

				count++;
			}
		}
	}

	void forward()
	{
		makeInputVectorFromHistory(0, old_input_vector_);
		nn_.setInputVector(old_input_vector_);
		nn_.feedForward();
	}

	// push back this to history
	void recordHistory(const VectorND<float>& state_vector, const float& reward, const int& choice, const VectorND<float>& q_values)
	{
		//TODO: this makes slower
		memory_.append(state_vector, choice, reward, q_values);
	}

	void makeInputVectorFromHistory(const int& ix_from_end, VectorND<float>& input)
	{
		for (int r = 0, count = 0; r < num_input_histories_; r++, count += num_state_variables_)
		{
			const VectorND<float> &state_vector =
				memory_.getStateVectorFromLast(ix_from_end - r);
			//history_.getValue(history_.getLastIndex() - r + ix_from_end).state_vector_;

			input.copyPartial(state_vector, count, 0, num_state_variables_);
		}
	}
};