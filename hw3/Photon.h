//#pragma once
//#pragma once
//#include <Eigen/Dense>
//#include <vector>
//#include <algorithm>
//#include <queue>
//
//struct photon
//{
//	Eigen::Vector3f position;
//	Eigen::Vector3f direction; //incident direction
//	Eigen::Vector3f power;
//	bool flag; //flag used in kdtree
//};
//
//struct node
//{
//	photon *photon;
//	node* left;
//	node* right;
//	int axis;
//};
//
//class KDtree
//{
//public:
//	int stored_photons;
//	photon *photons;
//
//	KDtree();
//	~KDtree();
//	void balance();
//	node* construct_kdtree(std::vector<photon> points, int depth);
//	bool sort_x(photon &point1, photon &point2);
//	bool sort_y(photon &point1, photon &point2);
//	bool sort_z(photon &point1, photon &point2);
//
//private:
//};
