/*
 *	Authored 2022, Greg Brooks, Damien Zufferey.
 *
 *	Copyright (c) 2020--2022 Signaloid.
 *	All rights reserved.
 */


#pragma once

#include <array>

/*
 *	Dimension of the state-space
 */
constexpr int NSTATE = 3;

/*
 *	Dimension of the input-space
 */
constexpr int NINPUT = 2;

/*
 *	State array
 */
typedef std::array<float, NSTATE> state;

/*
 *	Control inputs array
 */
typedef std::array<float, NINPUT> input;

/**
 *	@brief Integration using Heun's method.
 *
 *	@param xNew       : Array to store the return values (new state).
 *	@param x          : Array holding the previous state.
 *	@param uNew       : Wheel speed measurements for the new timestep.
 *	@param u          : Wheel speed measurements for the previous timestep.
 *	@param dt         : Duration/time period.
 *	@param trackWidth : Distance between the two wheels on the robot.
 */
void
integrate(
	state & xNew,
	const state & x,
	const input & uNew,
	const input & u,
	const float dt,
	const float trackWidth
);
