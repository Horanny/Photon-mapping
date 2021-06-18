#pragma once
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <queue>
#include <math.h>
#include <iostream>
#include <ANN/ANN.h>
#include  "constant.hpp"
#include "ray.hpp"
using namespace std;

struct Photon {
	Eigen::Vector3f pos; //photon position
	Eigen::Vector3f dir; //incoming direction
	Eigen::Vector3f power; //photon power	
	int flag;
};

class PhotonMapper {
public:
	PhotonMapper();
	~PhotonMapper();

	void storePhoton(Photon &p);
	void storeCausticPhoton(Photon &p);
	void LoadToKDtree();
	
	Eigen::Vector3f trace(Ray ray, int depth);
	//given a ray, collects the photons around the intersection point 
	Eigen::Vector3f gatherPhotons(Ray ray);


	vector<Photon> photonMap;
	int num;
	ANNpointArray dataPoints; 
	ANNidxArray near_neighbour_idx; //near neighbor indices
	ANNdistArray near_neighbour_dists; // near neighbour distances
	ANNkd_tree* kdTree; //kd tree

	vector<Photon>causticsMap;

};

