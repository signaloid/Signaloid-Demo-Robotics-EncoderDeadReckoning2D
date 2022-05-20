/*
 *	Authored 2022, Greg Brooks.
 *
 *	Copyright (c) 2020--2022 Signaloid.
 *	All rights reserved.
 */
#include <algorithm>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include "userInput.hpp"

#include <iostream>

/**
 *	@brief Print usage instructions to stdout.
 *
 *	@param defaultParameters : Default user parameters to include in the printout.
 */
static void
printUsage(const UserParameters & defaultParameters)
{
	printf("Encoder based dead reckoning state estimation\n");
	printf("\n");
	printf("    encoder-dead-reckoning [-i file] [-s state] [-t float] [-w float]\n");
	printf("\n");
	printf("    -i file: path to CSV file containing encoder timer measurements\n");
	printf("\n");
	printf("    -k float: constant to divide by timer measurements to obtain wheel speed\n");
	printf("\n");
	printf("    -m float: maximum encoder timer count threshold, above which wheel speed is "
		"rounded to zero\n");
	printf("\n");
	printf("    -s (float:)*float: initial state (default: 0:0:0)\n");
	printf("\n");
	printf("    -t float: time step for the discretization of the dynamics (default: %fs)\n",
		defaultParameters.timestep);
	printf("\n");
	printf("    -w float: robot track width i.e. distance between wheels (default: %fs)\n",
		defaultParameters.trackWidth);
	printf("\n");
}

/**
 *	@brief Parse float from char array.
 *
 *	@param arg : Pointer to char array.
 *	@param opt : Command line option associated with this char array.
 *	@return float : Parsed float
 */
static float
parseFloat(char * arg, const char * opt)
{
	float f;

	errno = 0;
	f = strtof(arg, NULL);

	if (errno != 0)
	{
		perror(opt);
		exit(EXIT_FAILURE);
	}

	return f;
}

/**
 *	@brief Parse int from string.
 *
 *	@param arg : Reference to string to parse.
 *	@return float : Parsed float
 */
static float
parseIntFromString(const std::string & arg)
{
	constexpr int base = 10;
	int i;

	errno = 0;
	i = static_cast<int>(strtol(arg.c_str(), nullptr, base));

	if(errno != 0)
	{
		throw std::runtime_error("Failed to parse int from string");
	}

	return i;
}

/**
 *	@brief Read encoder inputs from the specified file.
 *
 *	@param arg : Pointer to char array containing file path.
 *	@return std::vector<input> : The encoder inputs extracted from the file.
 */
static std::vector<input>
readEncoderInputs(char * arg)
{
	std::string line;
	std::string inputA;
	std::string inputB;
	int iInputA;
	int iInputB;
	std::vector<input> newEncoderVector;

	errno = 0;
	std::ifstream fin(arg);

	if(!fin)
	{
		throw std::runtime_error("Could not open file");
	}

	while(std::getline(fin, line))
	{
		std::stringstream stringStreamLine(line);
		if(
			!getline(stringStreamLine, inputA, ',') ||
			!getline(stringStreamLine, inputB)
		)
		{
			throw std::runtime_error("CSV file is malformed");
		}

		try
		{
			iInputA = parseIntFromString(inputA);
			iInputB = parseIntFromString(inputB);
		}
		catch(const std::runtime_error & e)
		{
			throw std::runtime_error("CSV file is malformed");
		}
		const input newInput = {
			static_cast<float>(iInputA),
			static_cast<float>(iInputB)
		};
		newEncoderVector.push_back(newInput);
	}

	fin.close();

	return newEncoderVector;
}

/**
 *	@brief Parse a positive float from optarg.
 *
 *	@param option : Command line option associated with the float.
 *	@return float : Parsed float.
 */
static float
handleInputArgPositiveFloat(const char * option)
{
	std::ostringstream opt;
	float value;

	opt << "Option '-" << option << "'";
	value = parseFloat(optarg, opt.str().c_str());

	if(value < 0)
	{
		fprintf(
			stderr,
			"Option -%s needs a positive argument, not %f\n",
			option,
			value
		);
		exit(EXIT_FAILURE);
	}

	return value;
}

/**
 *	@brief Extract encoder inputs from optarg.
 *
 *	@param inputVector : Reference to vector to store results.
 */
static void
handleInputArgEncoderInputs(std::vector<input> & inputVector)
{
	inputVector = readEncoderInputs(optarg);
}

/**
 *	@brief Parse the wheel constant from optarg.
 *
 *	@return float : Extracted wheel constant.
 */
static float
handleInputArgWheelConstant(void)
{
	return parseFloat(optarg, "Option '-k'");
}

/**
 *	@brief Parse the maximum encoder timer count from optarg.
 *
 *	@return float : Extracted maximum encoder timer count.
 */
static float
handleInputArgMaxTimerCount(void)
{
	return handleInputArgPositiveFloat("m");
}

/**
 *	@brief Parse the initial robot state vector from optarg.
 *
 *	@param initialState : Reference to location to store result.
 */
static void
handleInputArgInitialState(state & initialState)
{
	char * pstr = optarg;
	
	errno = 0;

	for (int i = 0; i < NSTATE; i++)
	{
		initialState.at(i) = strtof(pstr, &pstr);
		if (errno != 0)
		{
			perror("initial state");
			exit(EXIT_FAILURE);
		}
		if (i < NSTATE - 1)
		{
			if (*pstr != ':')
			{
				fprintf(stderr, "initial state: bad format %s\n", optarg);
				exit(EXIT_FAILURE);
			}
			++pstr;
		}
	}
}

/**
 *	@brief Extract timestep user input from optarg.
 *
 *	@return float : Timestep provided by user.
 */
static float
handleInputArgTimestep(void)
{
	return handleInputArgPositiveFloat("t");
}

/**
 *	@brief Extract track width input from optarg.
 *
 *	@return float : Track width provided by user.
 */
static float
handleInputArgTrackWidth(void)
{
	return handleInputArgPositiveFloat("w");
}

void
getUserInputs(UserParameters & parameters, int argc, char * argv[])
{
	UserParameters newParameters = parameters;
	int c;

	while ((c = getopt(argc, argv, "i:k:m:s:t:w:")) != -1)
	{
		switch (c)
		{
		case 'i':
			handleInputArgEncoderInputs(newParameters.inputVector);
			break;
		case 'k':
			newParameters.wheelConstant = handleInputArgWheelConstant();
			break;
		case 'm':
			newParameters.maximumEncoderTimerCount = handleInputArgMaxTimerCount();
			break;
		case 's':
			handleInputArgInitialState(newParameters.initialState);
			break;
		case 't':
			newParameters.timestep = handleInputArgTimestep();
			break;
		case 'w':
			newParameters.trackWidth = handleInputArgTrackWidth();
			break;
		default:
			fprintf(stderr, "arguments malformed\n");
			printUsage(parameters);
			exit(EXIT_FAILURE);
		}
	}

	parameters = newParameters;
}
