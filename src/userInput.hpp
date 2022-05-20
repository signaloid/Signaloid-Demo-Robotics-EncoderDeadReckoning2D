/*
 *	Authored 2022, Greg Brooks.
 *
 *	Copyright (c) 2020--2022 Signaloid.
 *	All rights reserved.
 */

#pragma once

#include <vector>
#include "Integrate.hpp"

/**
 *	@brief Parameters supplied via stdin.
 *
 */
typedef struct UserParameters
{
	/*
	 *	Vector containing encoder inputs
	 */
	std::vector<input> inputVector;
	/*
	 *	Initial robot state at time t=0
	 */
	state initialState;
	/*
	 *	Maximum encoder timer count
	 */
	float maximumEncoderTimerCount;
	/*
	 *	Timestep between encoder measurements
	 */
	float timestep;
	/*
	 *	Distance between robot wheels
	 */
	float trackWidth;
	/*
	 *	Constant used to convert encoder timer counts to wheel speeds
	 */
	float wheelConstant;
} UserParameters;

/**
 *	@brief Populate a UserParameters struct from argc and argv
 *
 *	@param parameters : Reference to struct to populate
 *	@param argc       : Argc
 *	@param argv       : Pointer to argv
 */
void
getUserInputs(UserParameters & parameters, int argc, char * argv[]);

