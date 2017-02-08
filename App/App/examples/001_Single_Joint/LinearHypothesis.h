#pragma once

class LinearHypothesis {
public:
	float a, b;

	LinearHypothesis()
		: a(0.0f), b(0.0f)
	{}

	float getY(const float& x_input) {
		return a * x_input + b;
	}
};