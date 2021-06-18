//#include "Photon.h"
//
//KDtree::KDtree()
//{
//}
//
//KDtree::~KDtree()
//{
//}
//
//void KDtree::balance()
//{
//
//}
//
//node* KDtree::construct_kdtree(std::vector<photon> points, int depth)
//{
//	if (points.size() == 0)
//		return;
//
//	//find current dimension, e.g. x,y,z-axis
//	int dim = depth % points[0].position.size();
//
//	//find the medium point in the chosen dimension
//	int medium_index = points.size() / 2;
//
//	//sort the points
//	if (dim == 0)
//		//std::nth_element(points.begin(), points.begin() + medium_index, points.end());
//		sort(points.begin(), points.end(), sort_x);
//	else if (dim == 1)
//		sort(points.begin(), points.end(), sort_y);
//	else if (dim == 2)
//		sort(points.begin(), points.end(), sort_z);
//
//	//construct left and right subvectors
//	std::vector<photon> left, right;
//	for (int i = 0; i < medium_index; i++)
//		left.push_back(points[i]);
//	for (int i = medium_index + 1; i < points.size(); i++)
//		right.push_back(points[i]);
//
//	points.clear();
//
//	//construct a node structure for the medium point and assign properties to it
//	node* medium_node;
//	medium_node->photon = &points[medium_index];
//	medium_node->left = construct_kdtree(left, depth + 1);
//	medium_node->right = construct_kdtree(right, depth + 1);
//
//	return medium_node;
//}
//
//#pragma region sort x,y,z-axis
//
//bool KDtree::sort_x(photon & point1, photon & point2)
//{
//	return point1.position[0] < point2.position[0];
//}
//
//bool KDtree::sort_y(photon & point1, photon & point2)
//{
//	return point1.position[1] < point2.position[1];
//}
//
//bool KDtree::sort_z(photon & point1, photon & point2)
//{
//	return point1.position[2] < point2.position[2];
//}
//#pragma endregion