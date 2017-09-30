# CarND2-P3 Kidnapped Vehicle

## Description

**This my 3rd project result of Udacity self-driving car nanodegree (CarND) term 2. It's required to localize a "kidnapped" (i.e. loss localization) vehicle given a map, a (noisy) GPS estimation of vehicle initial location, (noisy) range sensing and control data. In this project, 2 dimensional  particle filter is implemented and a simulator is provided to visualize particle filter localization result.**

**The following demonstrates particle filter localization result :** 

![alt text][image1]

* Udacity self-driving car nanodegree (CarND) :

  https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
  
* Udacity self-driving car nanodegree term 2 simulator :

  https://github.com/udacity/self-driving-car-sim/releases/

The goals / steps of this project are shown as below:

[//]: # (Image References)
[image1]: ./images/particle_filter.gif
[image2]: ./images/flowchart.png

![alt text][image2]

**File structure:**

* The measurement data receiving and connection to simulator is in C++ file `./src/main.cpp`.

* The particle filter initialization, predict and update is called in C++ file `./src/particle_filter.cpp`.

## Usage
* `./clean.sh` 
* `./build.sh`
* `./run.sh`
* Download simulator `term2_sim.app` (if in OSX) and open it. Click `play!` bottom, select Project 3: Kidnapped Vehicle, and press `Start` bottom to start.

## Particle Filter

Particle filter is a Bayes filter implementation based on Monte-Carlo method. It first generate several particles to represent the robot localization, make prediction based on control data and robot motion model, update localization using sensing data (e.g. range sensor), resample particles according to weights estimated based on Gaussian distribution, and repeat to prediction step again. 

typical technique used to fuse multiple sensing data and get a more accurate result (state) based on linear assumption and Bayes rule. 
Unlike (extended) Kalman filter which assumes measurement to be linear, unscented Kalman filter can be applied to both linear and nonlinear case. 




