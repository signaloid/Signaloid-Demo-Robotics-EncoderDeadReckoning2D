/*
 *	Authored 2022, Greg Brooks, Damien Zufferey.
 *
 *	Copyright (c) 2020--2022 Signaloid.
 *	All rights reserved.
 */

#include <iostream>
#include "Integrate.hpp"
#include "userInput.hpp"
#include "uncertain.h"

/**
 *	@brief Print robot state to stdout.
 *
 *	@param arg : State to print.
 */
static void
printState(state arg)
{
	std::cout << "x=" << arg.at(0) << ", ";
	std::cout << "y=" << arg.at(1) << ", ";
	std::cout << "θ=" << arg.at(2) << std::endl;
}

/**
 *	@brief Convert raw encoder timer measurements to wheel speeds, tracking uncertainty
 *	       introduced by the quantisation noise resulting from discrete integer digital timer
 *	       counts.
 *
 *	@param timerInput        : Raw encoder timer input.
 *	@param wheelConstant     : Constant to use when converting measured time to wheel speed
 *	                           (speed = constant / time).
 *	@param maximumTimerCount : Maximum timer count, above which wheel speed will be set to zero.
 *	@return input : Wheel speeds (right wheel, left wheel)
 */
static input
convertTimerInputsToWheelSpeeds(
	const input timerInput,
	const float wheelConstant,
	const float maximumTimerCount)
{
	input wheelSpeeds;

	for(size_t i = 0; i < timerInput.size(); i++)
	{
		if(timerInput.at(i) >= maximumTimerCount)
		{
			wheelSpeeds.at(i) = libUncertainFloatUniformDist(
				0.0f,
				wheelConstant / maximumTimerCount
			);
		}
		else if(timerInput.at(i) <= 0.0f - maximumTimerCount)
		{
			wheelSpeeds.at(i) = libUncertainFloatUniformDist(
				(0.0f - wheelConstant) / maximumTimerCount,
				0.0f
			);
		}
		else if(timerInput.at(i) < 1.0f && timerInput.at(i) >= 0.0f)
		{
			wheelSpeeds.at(i) = libUncertainFloatUniformDist(0.1, 0.5);
		}
		else if(timerInput.at(i) > -1.0f && timerInput.at(i) < 0.0f)
		{
			wheelSpeeds.at(i) = libUncertainFloatUniformDist(-0.5, -0.1);
		}
		else
		{
			float uncertainTimerCount = libUncertainFloatUniformDist(
				timerInput.at(i) - 0.5f,
				timerInput.at(i) + 0.5f
			);

			wheelSpeeds.at(i) = wheelConstant / uncertainTimerCount;
		}
	}

	return wheelSpeeds;
}

int
main(int argc, char * argv[])
{
	/*
	 *	Populate default parameters
	 */
	UserParameters parameters = {
		.initialState             = {0.0, 0.0, 0.0},
		.maximumEncoderTimerCount = 65535,
		.timestep                 = 0.1f,
		.trackWidth               = 1.0f,
		.wheelConstant            = 360.0f,
	};
	state oldState;
	state newState = {0.0f, 0.0f, 0.0f};

	/*
	 *	Populate input vector with default values (robot drives around in a circle)
	 */
	for(size_t i = 0; i < 81; i++)
	{
		parameters.inputVector.push_back({230, 460});
	}

	getUserInputs(parameters, argc, argv);

	if(parameters.inputVector.size() < 2)
	{
		throw std::runtime_error(
			"Need at least two successive encoder measurements to perform numerical"
			"integration"
		);
	}

	oldState = parameters.initialState;
	std::cout << "t=0: ";
	printState(oldState);

	for(size_t i = 0; i < parameters.inputVector.size() - 1; i++)
	{
		integrate(
			newState,
			oldState,
			convertTimerInputsToWheelSpeeds(
				parameters.inputVector.at(i),
				parameters.wheelConstant,
				parameters.maximumEncoderTimerCount),
			convertTimerInputsToWheelSpeeds(
				parameters.inputVector.at(i+1),
				parameters.wheelConstant,
				parameters.maximumEncoderTimerCount),
			parameters.timestep,
			parameters.trackWidth
		);

		oldState = newState;
	}

	std::cout << "t=" << parameters.inputVector.size() - 1 << ": ";
	printState(newState);

	return 0;
}
