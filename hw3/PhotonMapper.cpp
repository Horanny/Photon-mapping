#include "PhotonMapper.h"

PhotonMapper::PhotonMapper()
{
	num = 0;
	near_neighbour_idx = new ANNidx[ESTIMATE];
	near_neighbour_dists = new ANNdist[ESTIMATE];
}

PhotonMapper::~PhotonMapper()
{
	delete[] near_neighbour_idx;
	delete[] near_neighbour_dists;
	delete kdTree;
}

void PhotonMapper::storePhoton(Photon & p)
{
	photonMap.push_back(p);
}

void PhotonMapper::storeCausticPhoton(Photon & p)
{
	causticsMap.push_back(p);
}

void PhotonMapper::LoadToKDtree()
{
	dataPoints = annAllocPts(photonMap.size(), 3);
	for (int i = 0; i < photonMap.size(); i++)
	{
		Photon p = photonMap[i];
		dataPoints[num][0] = p.pos[0];
		dataPoints[num][1] = p.pos[1];
		dataPoints[num][2] = p.pos[2];

		num++;
	}

	kdTree = new ANNkd_tree(dataPoints, num, 3);
}

Eigen::Vector3f PhotonMapper::trace(Ray ray, int depth)
{

	return Eigen::Vector3f();
}

Eigen::Vector3f PhotonMapper::gatherPhotons(Ray ray)
{
	return Eigen::Vector3f();
}

