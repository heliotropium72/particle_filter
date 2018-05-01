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

	num_particles = 1000;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	default_random_engine gen;
	for (int i = 0; i < num_particles; ++i) {
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1;

		particles.push_back(p);
		weights.push_back(1);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;

	for (int i = 0; i < num_particles; ++i) {
		double new_x, new_y, new_theta;

		// (Bicycle) motion model
		if (yaw_rate == 0) {
			new_x = particles[i].x + velocity * sin(particles[i].theta) * delta_t;
			new_y = particles[i].y + velocity * cos(particles[i].theta) * delta_t;
			new_theta = particles[i].theta;
		}
		else {
			new_x = particles[i].x + velocity/yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
			new_y = particles[i].y + velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
			new_theta = particles[i].theta + yaw_rate * delta_t;
		}

		// Add noise
		normal_distribution<double> dist_x(new_x, std_pos[0]);
		normal_distribution<double> dist_y(new_y, std_pos[1]);
		normal_distribution<double> dist_theta(new_theta, std_pos[2]);

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	for (size_t i = 0; i < observations.size(); i++) {
		double distance;
		double distance_min = 9999;
		observations[i].id = 0; // default to first entry
		// calculate eucledian distance to every landmark and replace id if smaller
		for (size_t k = 0; k < predicted.size(); k++) {
			distance = sqrt(pow(observations[i].x - predicted[i].x, 2) + pow(observations[i].y - predicted[i].y, 2));
			if (distance < distance_min) {
				distance_min = distance;
				observations[i].id = predicted[k].id
			}
		}

	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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


	//const for multi-variate gauss
	double mvg_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
	double mvg_denom_x = 1 / (2 * std_landmark[0] * std_landmark[0]);
	double mvg_denom_y = 1 / (2 * std_landmark[1] * std_landmark[1]);

	double new_w = 1.0;

	for (int i = 0; i < num_particles; ++i) {

		// select landmarks within sensor range
		std::vector<LandmarkObs> predicted;
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			// Euclidean distance between each landmark and particle position
			double distance = sqrt( pow(map_landmarks.landmark_list[j].x_f - particles[i].x, 2) +
				pow(map_landmarks.landmark_list[j].y_f - particles[i].y, 2) );

			if (distance < sensor_range){
				LandmarkObs observable;
				observable.id = j;
				observable.x = map_landmarks.landmark_list[j].x_f;
				observable.y = map_landmarks.landmark_list[j].y_f;
				predicted.push_back(observable);
			}	
		}

		// Coordinate transformation of obsevations from car's system to map's system
		// rotation with particles[i].theta and translation by particles[i].x and particles[i].y
		vector<LandmarkObs> observations_trans;
		double ct = cos(particles[i].theta);
		double st = sin(particles[i].theta);
		for (size_t k = 0; k < observations.size(); k++) {
			// Coordinate transformation
			LandmarkObs obs_trans;
			obs_trans.x = observations[k].x *ct - observations[k].y *st + particles[i].x; // check if theta has an offset and direction of translataion
			obs_trans.y = observations[k].x *st + observations[k].y *ct + particles[i].y;
			observations_trans.push_back(obs_trans);
		}

		dataAssociation(predicted, observations_trans);

		// Save results in class for visualisation
		vector<int> associations;
		vector<double> sense_x;
		vector<double> sense_y;

		for (size_t k = 0; k < observations_trans.size(); k++) {
			// Find the associated landmark
			int id = observations_trans[k].id;

			// Multi-variant Gaus for one particle
			double exponent = - pow((observations_trans[k].x - map_landmarks.landmark_list[id].x_f), 2) * mvg_denom_x +
								pow((observations_trans[k].y - map_landmarks.landmark_list[id].y_f), 2) * mvg_denom_y;
			double probability = mvg_norm * exp(exponent);

			if (probability > 0) { // in case of rounding errors, don't set total weight to zero
				new_w *= probability;
			}
			
			associations.push_back(id + 1);
			sense_x.push_back(observations_trans[i].x);
			sense_y.push_back(observations_trans[i].y);
		}

		// Visualisation
		particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y);

		// Update weight
		particles[i].weight = new_w;
		weights[i] = new_w;

	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resample_particles;
	for (int i = 0; i > num_particles; ++i) {
		resample_particles.push_back(particles[distribution(gen)]);
	}
	particles = resample_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
