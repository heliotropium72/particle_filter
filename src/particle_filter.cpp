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

	num_particles = 100;

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
		p.weight = 1.0;
		particles.push_back(p);
	}

	is_initialized = true;

	std::cout << "Initialized particle filter with " << num_particles << " particles" << std::endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	//int id = 0;
	//std::cout << "Prediction for dt = " << delta_t << " with v = " << velocity << " m/s, dtheta/dt = " << yaw_rate << " rad/s" << std::endl;
	//std::cout << "Before \t(" << particles[id].x  << ", " << particles[id].y << "), yaw = " << particles[id].theta << std::endl;

	
	// Additive sensor noise, distributions are the same for all particles
	default_random_engine gen;
	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> dist_theta(0.0, std_pos[2]);

	double delta_theta = yaw_rate * delta_t;

	for (int i = 0; i < num_particles; i++) {
		//std::cout << "Particle " << particles[i].id << " at (" << particles[i].x << ", " << particles[i].y << ")" << std::endl;
		double new_x, new_y, new_theta;

		// (Bicycle) motion model
		if (fabs(yaw_rate) < 0.001) {
			new_x = particles[i].x + velocity * cos(particles[i].theta) * delta_t;
			new_y = particles[i].y + velocity * sin(particles[i].theta) * delta_t;
			new_theta = particles[i].theta;
		}
		else {
			new_x = particles[i].x + velocity/yaw_rate * (sin(particles[i].theta + delta_theta) - sin(particles[i].theta));
			new_y = particles[i].y + velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + delta_theta));
			new_theta = particles[i].theta + delta_theta;
		}

		// Add noise
		particles[i].x = new_x + dist_x(gen);
		particles[i].y = new_y + dist_y(gen);
		particles[i].theta = new_theta + dist_theta(gen);
	}

	//std::cout << "After \t(" << particles[id].x << ", " << particles[id].y << "), yaw = " << particles[id].theta << std::endl;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	double distance;

	for (size_t i = 0; i < observations.size(); i++) { 
      double distance_min = 99999.0;
      observations[i].id = -1; // default to first entry
      // calculate eucledian distance to every landmark and replace id if smaller
      for (size_t k = 0; k < predicted.size(); k++) {
		distance = dist(observations[i].x, observations[i].y, predicted[k].x, predicted[k].y);
	    if (distance < distance_min) {
		  distance_min = distance;
		  observations[i].id = predicted[k].id;
	    }
      }
	//std::cout << "Observation " << i << " associates to " << observations[i].id << ", distance: " << distance_min << std::endl;
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
	double mvg_norm = 1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]);
	double mvg_denom_x = 1.0 / (2.0 * std_landmark[0] * std_landmark[0]);
	double mvg_denom_y = 1.0 / (2.0 * std_landmark[1] * std_landmark[1]);

	// For each particle
	for (int i = 0; i < num_particles; i++) {

		//std::cout << "Particle: " << particles[i].x << " , " << particles[i].y << " , " << particles[i].theta << std::endl;
		// select landmarks within sensor range
		std::vector<LandmarkObs> predicted;
		for (size_t j = 0; j < map_landmarks.landmark_list.size(); j++) {
			// Euclidean distance between each landmark and particle position
			double distance = dist(map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f,
				particles[i].x, particles[i].y);

			if (distance <= sensor_range) {
				LandmarkObs observable;
				observable.id = map_landmarks.landmark_list[j].id_i;
				observable.x = map_landmarks.landmark_list[j].x_f;
				observable.y = map_landmarks.landmark_list[j].y_f;
				predicted.push_back(observable);
			}
		}
		//std::cout << "Landmarks within range: " << predicted.size() << std::endl;

		// Coordinate transformation of observations from car's system to map's system
		// rotation with particles[i].theta and translation by particles[i].x and particles[i].y
		vector<LandmarkObs> observations_trans;
		double ct = cos(particles[i].theta);
		double st = sin(particles[i].theta);
		for (size_t k = 0; k < observations.size(); k++) {
			// Coordinate transformation
			LandmarkObs obs_trans;
			obs_trans.id = observations[k].id;
			obs_trans.x = observations[k].x *ct - observations[k].y *st + particles[i].x;
			obs_trans.y = observations[k].x *st + observations[k].y *ct + particles[i].y;
			observations_trans.push_back(obs_trans);
			//std::cout << "original: " << observations[k].x << " , " << observations[k].y << std::endl;
			//std::cout << "transformed: " << obs_trans.x << " , " << obs_trans.y << std::endl;
		}

		dataAssociation(predicted, observations_trans);

		// Calculate the weight by multiplication of single observation probabilities
		particles[i].weight = 1.0;
		for (size_t k = 0; k < observations_trans.size(); k++) {
			// Find the associated landmark
			int id = observations_trans[k].id;
			int index;
			// Find the index of the associated landmark
			for (size_t j = 0; j < predicted.size(); j++) {
				if (id == predicted[j].id) {
					index = j;
				}
			}
			// Multi-variant Gaus for one particle
			double exponent = pow((observations_trans[k].x - predicted[index].x), 2) * mvg_denom_x +
				pow((observations_trans[k].y - predicted[index].y), 2) * mvg_denom_y;
			double probability = mvg_norm * exp(-1 * exponent);

			if (probability != 0){
				particles[i].weight *= probability;
			}
		}
		// Update weight
		weights.push_back(particles[i].weight);
		//std::cout << "Particle " << particles[i].id << " at (" << particles[i].x << ", " << particles[i].y << ")" << " weights " << particles[i].weight << std::endl;

		// Visualisation
		vector<int> associations;
		vector<double> sense_x;
		vector<double> sense_y;
		for (size_t k = 0; k < observations_trans.size(); k++) {
		  associations.push_back(observations_trans[k].id);
		  sense_x.push_back(observations_trans[k].x);
		  sense_y.push_back(observations_trans[k].y);
	    }
		particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y);	
	}	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	
	// use the weights to build a discrete distribution giving for every i its probability
	// distribution(gen) picks an index i with probability wi
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resample_particles;
	for (int i = 0; i < num_particles; ++i) {
		resample_particles.push_back(particles[distribution(gen)]);
	}

	particles = resample_particles;
	weights.clear();
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
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
