[<img src="https://assets.signaloid.io/add-to-signaloid-cloud-logo-dark-v6.png#gh-dark-mode-only" alt="[Add to signaloid.io]" height="30">](https://signaloid.io/repositories?connect=https://github.com/signaloid/Signaloid-Demo-Robotics-EncoderDeadReckoning2D#gh-dark-mode-only)
[<img src="https://assets.signaloid.io/add-to-signaloid-cloud-logo-light-v6.png#gh-light-mode-only" alt="[Add to signaloid.io]" height="30">](https://signaloid.io/repositories?connect=https://github.com/signaloid/Signaloid-Demo-Robotics-EncoderDeadReckoning2D#gh-light-mode-only)



# Encoder-Based Dead Reckoning State Estimation for a Two-Wheeled Robot
This example demonstrates a simple encoder-based dead reckoning algorithm for a two-wheeled robot moving in two dimensions. In this example, encoders attached to each wheel of the robot provide wheel speed measurements. The code in the example numerically integrates the measurements over time to estimate the position and orientation of the robot.

Dead reckoning algorithms are subject to cumulative errors, causing the accuracy of the state estimate to decrease over time. Signaloid's uncertainty-tracking processor is able to track this uncertainty as the algorithm runs. In this example, the source of uncertainty in the state estimate is the quantisation error resulting from digitised encoder measurements. The implementation in this example models the error as additive uniformly-distributed noise.


![diagram](https://user-images.githubusercontent.com/8598997/169335335-87138975-d5c6-4b47-8896-42b66f630e71.svg)


The diagram above shows a model of the two-wheeled robot. The encoders measure speed at each wheel (`v_L(t)` and `v_R(t)`), allowing the implementation in the example to calculate the overall linear and angular velocities of the robot. The robot's state is defined in terms of its position (`x(t)` and `y(t)`) and orientation `θ(t)`. The track width, `w`, is the distance between the two wheels.

## Inputs
In this example, encoder (tachometer) measurements are provided in the form of a digital timer count, i.e., an integer value representing time elapsed between two successive tachometer pulses. This reflects a physical system where the tachometer pulses are used to trigger a digital timer, for example, a timer peripheral within a microcontroller. The speed of the robot measured at the corresponding wheel can then be calculated from:
```
        angle of rotation * wheel radius     constant
speed = -------------------------------- = ------------
                time elapsed               time elapsed
```
where the `constant` combines the resolution of the encoder (angle rotated between pulses) and the wheel radius.

This example tracks the quantisation error in the timer measurements, tracking this uncertainty through to the state estimate.

The command line arguments for this example are:
-   `-i` : Path to CSV file containing encoder measurements, the expected contents of this file are described in detail below.
-   `-k` : Constant to divide by timer count to obtain wheel speed.
-   `-m` : Maximum timer count threshold, above which wheel speed is rounded down to zero.
-   `-s` : Initial state estimate, in the format: `<position x>:<position y>:<orientation angle>`.
-   `-t` : Timestep duration in seconds (time between successive encoder measurements).
-   `-w` : Robot track width (the distance between the two wheels).

If no command-line arguments are specified, the program will run with a default set of parameters simulating the robot driving around in a circle to end up back at the origin.

### Command Line Argument Example
This program can be run in Signaloid's C0 Cloud Developer Platform using these command line arguments:
```
-i input-circle.csv -k 360 -m 65535 -s 0:0:0 -t 0.1 -w 1.0
```

These are the default values used if no command-line arguments are specified.

### Input CSV File
The CSV file containing encoder measurements is expected to follow the format:
```
r(0), l(0)
r(1), l(1)
...
r(n), l(n)
```
where `r(t)` and `l(t)` are the encoder timer measurements for the right and left wheels respectively at timestep `t`.

When the program runs, it loops through each line in this file, updating the state estimate for each timestep until the end of the file (and hence the final robot state estimate) is reached.

## Generated Outputs
The program provides robot state estimates in the form:
```
⎛     Robot x position      ⎞
⎜     Robot y position      ⎟
⎝ Robot orientation angle θ ⎠
```
Each of the values in this state vector has an associated probability distribution representing the uncertainty in the state estimate.

As a simple example, when feeding in the inputs provided in `inputs/input-circle.txt`[^0] with the example command line arguments shown above, the output on the Signaloid Cloud Platform should look like:

<img width="600" alt="image" src="https://user-images.githubusercontent.com/8598997/169313961-f02f43a2-ce6b-4770-8622-c3500e3a436f.png">


Here, the initial robot state is for the robot to be located at position (0,0) with orientation 0. This initial state is assumed to be known with no uncertainty. The robot drives around in a circle, ending up back at (0,0) with orientation 0 as its final state.

Looking at the probability distributions associated with the final robot state, we can see that there is a greater level of uncertainty about the final state, introduced by the quantisation error in the timer measurements:

The final x-coordinate is -0.0334341 and its uncertainty distribution[^1] is:

<img src=https://user-images.githubusercontent.com/8598997/169314360-60a17d15-6090-42f5-a81b-1878b27413c7.png width=400>

<br/>

The final y-coordinate is 0.00037029 and its uncertainty distribution[^1] is:

<img src=https://user-images.githubusercontent.com/8598997/169314522-cf985bd6-0391-4b4c-8fc5-67c57a65b9e7.png width=400>

<br/>

The final θ is 6.26088 and its uncertainty distribution[^1] is:

<img src=https://user-images.githubusercontent.com/8598997/169314599-35e8e69b-e6e0-430e-9ce3-855efdaa7374.png width=400>

## Repository Tree Structure
```
.
├── README.md
├── inputs
|   ├── input-figure-eight.csv
│   └── input-circle.csv
└── src
    ├── config.mk
    ├── Integrate.cpp
    ├── Integrate.hpp
    ├── main.cpp
    ├── userInput.cpp
    └── userInput.hpp
```

<br/>
<br/>
<br/>

[^0]: This input file contains simulated encoder measurements for the case where the robot drives around in a circle to end up back at the origin. See also `inputs/input-figure-eight.csv` where the robot drives around in a figure of eight pattern, ending up back at the origin.
[^1]: Move your mouse over the number printed out on the Signaloid C0 Cloud Developer Platform to see the distribution.
