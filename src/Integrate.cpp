/*
 *	Authored 2022, Greg Brooks, Damien Zufferey.
 *
 *	Copyright (c) 2020--2022 Signaloid.
 *	All rights reserved.
 */
#define PI 3.14159265358979323846
#include <cmath>
#include <stdexcept>
#include "Integrate.hpp"

/**
 *	@brief Convert inputs from wheel speeds to vehicle speed (linear and angular).
 *
 *	@param trackWidth : Distance between wheels
 *	@param uIn        : Wheel speed ([right wheel, left wheel])
 *	@return input : Vehicle speed ([linear, angular])
 */
static input
convertInput(const float trackWidth, const input & uIn)
{
	float linearSpeed;
	float angularSpeed;
	input uOut;

	if(trackWidth == 0.0)
	{
		throw std::runtime_error("Track width cannot be zero");
	}

	linearSpeed  = (uIn[0] + uIn[1]) / 2.0f;
	angularSpeed = (uIn[0] - uIn[1]) / trackWidth;
	uOut = {linearSpeed, angularSpeed};

	return uOut;
}

/**
 *	@brief The derivatives of the dynamic.
 *
 *	@param dx : Return values
 *	@param x  : Current state
 *	@param u  : Inputs to the system
 */
static void
derivative(state & dx, const state & x, const input & u)
{
	const float linear_speed = u.at(0);
	const float angular_speed = u.at(1);
	const float theta = x.at(2);

	dx.at(0) = linear_speed * cos(theta);
	dx.at(1) = linear_speed * sin(theta);
	dx.at(2) = angular_speed;
}

void
integrate(
	state & xNew,
	const state & x,
	const input & uNew,
	const input & u,
	const float dt,
	const float trackWidth
)
{
	state k1;
	state xTmp;
	state k2;
	const input uNewConverted = convertInput(trackWidth, uNew);
	const input uConverted = convertInput(trackWidth, u);

	derivative(k1, x, uConverted);

	for (int i = 0; i < NSTATE; i++)
	{
		xTmp[i] = x[i] + (dt * k1[i]);
	}

	derivative(k2, xTmp, uNewConverted);

	for (int i = 0; i < NSTATE; i++)
	{
		xNew[i] = x[i] + dt * (k1[i] + k2[i]) / 2.0;
	}

	/*
	 *	Handle orientation wraparound
	 */
	while(xNew.at(2) > 2.0 * PI)
	{
		xNew.at(2) -= 2.0 * PI;
	}
	while(xNew.at(2) < 0.0f)
	{
		xNew.at(2) += 2.0 * PI;
	}
}
