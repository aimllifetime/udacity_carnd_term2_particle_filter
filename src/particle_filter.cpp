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
	// : Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	if(is_initialized == false){
		num_particles = 50;
		for(int i = 0; i < num_particles; i++){
			weights.push_back(1.0);
			Particle p;
			p.id = i;
			p.x  = dist_x(gen);
			p.y  = dist_y(gen);
			p.theta = dist_theta(gen);
			p.weight = 1.0;
			particles.push_back(p);
		}
		is_initialized = true;
		//cout << "ParticleFilter::init" << endl;
	}



}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	for(int i = 0; i < num_particles; i++){
		Particle p = particles[i];
		double x_pred;
		double y_pred;
		double theta_pred;
		double v_by_yaw_rate = velocity/yaw_rate;
		double yaw_rate_times_t = yaw_rate * delta_t;

		if(yaw_rate == 0){
			// xf = x0 + v(dt)(cos(theta0));
			// yf = y0 + v(dt)(sin(theta0));
			// thetaf = theta0;
			x_pred = p.x + velocity * delta_t * cos(p.theta);
			y_pred = p.y + velocity * delta_t * sin(p.theta);
			theta_pred = p.theta;
		}
		else{ // yaw_rate != 0
			// xf = x0 + v/thetadot*[sin(theta0 + thetadot * (dt) -sin(theta0)];
			// yf = y0 + v/thetadot * [ cos(theta0) - cos(theta0 + thetadot * dt)]
			// thetaf = theta0 + thetadot * dt
			x_pred = p.x + v_by_yaw_rate*(sin(p.theta + yaw_rate_times_t) -  sin(p.theta));
			y_pred = p.y + v_by_yaw_rate*(cos(p.theta) - cos(p.theta + yaw_rate_times_t));
			theta_pred = p.theta + yaw_rate_times_t;
		}
		// add the noise
		normal_distribution<double> dist_x(x_pred, std_pos[0]);
		normal_distribution<double> dist_y(y_pred, std_pos[1]);
		normal_distribution<double> dist_theta(theta_pred, std_pos[2]);

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		// cout << "particle i = " << i << endl;
		// cout << p.x << endl;
		// cout << p.y << endl;
		// cout << p.theta << endl;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// : Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// first remap the observations from vehicle's coordinate system to map system by using following matrix
	// homogenous transformation2d
	// | xm |   | cos(theta)  -sin(theta) xp |   |xc|
	// | ym | = | sin(theta)  cos(theta)  yp | x |yc|
	// | 1  |   |  0              0        1 |   | 1|
  //static int count = 0; //used for debug purpose for early terminating Run
  // for every particle, process the all observations rotation from vehicle coordinate to map coordinate
	//cout << "updateWeights is called with num_particles " << num_particles << endl;
	for(int i = 0; i < num_particles; i++){
		// cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
		// cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
		// cout << "updateWeights i = " << i << endl;
		std::vector<LandmarkObs> remapped_observations;
		Particle p = particles[i];
		for(int j = 0; j < observations.size(); j++){
			double xm = p.x + cos(p.theta) * observations[j].x - sin(p.theta) * observations[j].y;
			double ym = p.y + sin(p.theta) * observations[j].x + cos(p.theta) * observations[j].y;
			LandmarkObs obs;
			obs.x = xm;
			obs.y = ym;
			remapped_observations.push_back(obs);
		}

		// now we have remapped observed landmark. next is to find the nearest true landmark in map
		std::vector<int> closest_landmarks;
		std::vector<double> landmark_x;
		std::vector<double> landmark_y;
		for(int j = 0; j < remapped_observations.size(); j++){
			LandmarkObs ob = remapped_observations[j];

			double min_dist, dist_off;
			bool min_initialized = false;
			int  closest_landmark;
			// for each observation, to find the closet_landmark
			for(int l = 0; l < map_landmarks.landmark_list.size(); l++){
				dist_off = dist(ob.x, ob.y, map_landmarks.landmark_list[l].x_f, map_landmarks.landmark_list[l].y_f);
				if(dist_off > sensor_range){
					//cout << "dist_off is greater sensor range " << dist_off << endl;
					continue;
				}

				if(min_initialized == false){
					min_dist = dist_off;
					closest_landmark = l;
					min_initialized = true;
					continue;
				}

				if(dist_off < min_dist){
					min_dist = dist_off;
					closest_landmark = l;
				}
			}

			closest_landmarks.push_back(map_landmarks.landmark_list[closest_landmark].id_i);
			landmark_x.push_back(map_landmarks.landmark_list[closest_landmark].x_f);
			landmark_y.push_back(map_landmarks.landmark_list[closest_landmark].y_f);

		} // end of observation for(j = 0; j < remapped_observations.size(); j++){

		// now we can calcuate the weight of this particular particle using multi-variate Guassian
		std::vector<double> prob;
		double gauss_norm = (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));
		for(int j = 0; j < remapped_observations.size(); j++){
			LandmarkObs ob = remapped_observations[j];
			double exponent= ((ob.x - landmark_x[j])* (ob.x - landmark_x[j]))/(2 * std_landmark[0]*std_landmark[0]) +
			                 ((ob.y - landmark_y[j])* (ob.y - landmark_y[j]))/(2 * std_landmark[1]*std_landmark[1]);
			double weight_p = gauss_norm * exp( -exponent );

			prob.push_back(weight_p);
		}

		double weight_dot = 1.0;
		if(prob.size() >= 1){
			for(int p = 0; p < prob.size(); p++){
			  weight_dot = weight_dot * prob[p];
			}
			weights[i] = weight_dot;
			particles[i].weight = weight_dot;
		} else{
			weights[i] = 0.0;
			particles[i].weight = 0.0;
		}
		//cout << "particle weight " << i << " = " << weights[i] << endl;

	}// end of i loop
	//count++;
	//if(count == 5) exit(0);
}

void ParticleFilter::resample() {
	// : Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());
	vector<Particle> resample_particles;
	//cout << "weight begin end" << weights.begin() << endl;
	for (int i = 0; i < num_particles; i++){
		//cout << "resample particle weight = " << particles[i].weight << " weights=" << weights[i] << endl;
		int index = distribution(gen);
		//cout << "particle " << index << "is selected" << endl;
		resample_particles.push_back(particles[index]);
	}
	particles = resample_particles;
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
