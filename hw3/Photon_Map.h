//#pragma once
//#include <Eigen/Dense>
//#include <vector>
//#include <algorithm>
//#include <queue>
//#include <math.h>
//#include <iostream>
//
//struct Photon {
//	Eigen::Vector3f pos; //photon position
//	Eigen::Vector3f dir; //incoming direction
//	Eigen::Vector3f power; //photon power	
//	int plane; //splitting plane for kd-tree
//};
//
//struct NearestPhotons {
//	int max;
//	int found;
//	bool got_heap;
//	Eigen::Vector3f pos;
//	float *dist2;
//	const Photon **index; //what's this? const photon 类型的指向index指针的指针？
//};
//
//class Photon_map {
//
//public:
//	Photon_map(int max_phot);
//	~Photon_map();
//
//	void store(const Eigen::Vector3f power,
//		const Eigen::Vector3f pos, 
//		const Eigen::Vector3f dir);
//
//	void scale_photon_power(const float scale); //1/(number of emitted photons)
//
//	void balance(void); //balance the kd tree before use
//
//	void irradiance_estimate(Eigen::Vector3f irradiance, //returned irradiance
//		const Eigen::Vector3f pos, //surface pos
//		const Eigen::Vector3f normal, //surface normal at pos
//		const float max_dist, //max distance to look for photons
//		const int nphotons)const; //number of photons to use
//
//	//np is used to locate the photons
//	//call with index = 1  ?????
//	void locate_photons(NearestPhotons *np, const int index)const;
//
//private:
//	void balance_segment(
//		Photon **pbal,
//		Photon **porg,
//		const int index,
//		const int start,
//		const int end
//	);
//
//	void median_split(
//		Photon **p,
//		const int start,
//		const int end,
//		const int median,
//		const int axis
//	);
//
//	Photon *photons;
//
//	int stored_photons;
//	int half_stored_photons;
//	int max_photons;
//	int prev_scale;
//
//	Eigen::Vector3f bbox_min;
//	Eigen::Vector3f bbox_max;
//};
