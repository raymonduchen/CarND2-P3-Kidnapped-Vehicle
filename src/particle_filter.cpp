/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	default_random_engine gen;
	normal_distribution<double> n_x(x, std[0]);
	normal_distribution<double> n_y(y, std[1]);
	normal_distribution<double> n_theta(theta, std[2]);

	// Generate particles
	num_particles = 100;

	for (int i = 0; i < num_particles; i++)
	{
		// Add particle and weight 
		particles.push_back(Particle{ i, n_x(gen), n_y(gen), n_theta(gen), 1.0 });
		weights.push_back(1.0);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	normal_distribution<double> n_x(0.0, std_pos[0]);
	normal_distribution<double> n_y(0.0, std_pos[1]);
	normal_distribution<double> n_theta(0.0, std_pos[2]);

	for (int i = 0; i < num_particles; i++)
	{
		double particle_x = particles[i].x;
		double particle_y = particles[i].y;
		double particle_theta = particles[i].theta;

		if (yaw_rate < 0.0001 & yaw_rate > -0.0001)
		{
			// Go straight
			particles[i].x = particle_x + velocity * delta_t * cos(particle_theta) + n_x(gen);
			particles[i].y = particle_y + velocity * delta_t * sin(particle_theta) + n_y(gen);
		}
		else
		{
			// Make turn
			particles[i].x = particle_x + velocity / yaw_rate * (sin(particle_theta + yaw_rate * delta_t) - sin(particle_theta)) + n_x(gen);
			particles[i].y = particle_y + velocity / yaw_rate * (cos(particle_theta) - cos(particle_theta + yaw_rate * delta_t)) + n_y(gen);
			particles[i].theta = particle_theta + yaw_rate * delta_t + n_theta(gen);
		}
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	for (int i = 0; i < observations.size(); i++)
	{
		LandmarkObs obs = observations[i];
		
		double min_dist = numeric_limits<double>::max();

		int association_id = -1;

		for (int j = 0; j < predicted.size(); j++)
		{
			LandmarkObs pred = predicted[j];
			double current_dist = dist (obs.x, obs.y, pred.x, pred.y);

			if (current_dist < min_dist)
			{
				min_dist = current_dist;
				association_id = pred.id;
			}
		}

		observations[i].id = association_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	for (int i = 0; i <num_particles; i++)
	{
		vector<LandmarkObs> preds;

		double particle_x = particles[i].x;
		double particle_y = particles[i].y;
		double particle_theta = particles[i].theta;

		for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			int landmark_id = map_landmarks.landmark_list[j].id_i;
			float landmark_x = map_landmarks.landmark_list[j].x_f;
			float landmark_y = map_landmarks.landmark_list[j].y_f;

			// check sensing is not out of range
			if (dist(landmark_x, landmark_y, particle_x, particle_y) <= sensor_range )
			{
				preds.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
			}
		}
		// Apply homogeneous transformation
		vector<LandmarkObs> transformed_obs;
		for (int j = 0; j < observations.size(); j++)
		{
			double transform_x = particle_x + cos(particle_theta)*observations[j].x - sin(particle_theta) * observations[j].y;
			double transform_y = particle_y + sin(particle_theta)*observations[j].x + cos(particle_theta) * observations[j].y;
			transformed_obs.push_back(LandmarkObs{observations[j].id, transform_x, transform_y});
		}

		dataAssociation(preds, transformed_obs);
	
		// Update weight using Multivariate-Gaussian
		particles[i].weight = 1.0;
		for (int j = 0; j < transformed_obs.size(); j++)
		{
			double pred_x, pred_y;
			int association_pred = transformed_obs[j].id;

			// find nearnest landmark
			for (int k = 0; k < preds.size(); k++)
			{
				if (preds[k].id == association_pred)
				{
					pred_x = preds[k].x;
					pred_y = preds[k].y;
				}
			}

			double std_x = std_landmark[0];
			double std_y = std_landmark[1];
			double obs_x = transformed_obs[j].x;
			double obs_y = transformed_obs[j].y;
			double obs_weight = (1/(2*M_PI*std_x*std_y)) * exp( -( pow(pred_x-obs_x,2)/(2*pow(std_x,2)) + (pow(pred_y-obs_y,2)/(2*pow(std_y,2)) )));

			particles[i].weight *= obs_weight;
		}
		weights[i] = particles[i].weight;
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	vector<Particle> res_particles;

	default_random_engine gen;
	discrete_distribution<int> index(weights.begin(), weights.end());
	for (int i = 0; i < num_particles; i++)
	{
		Particle res_particle = particles[index(gen)];
		res_particles.push_back(res_particle);
	}
	particles = res_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
