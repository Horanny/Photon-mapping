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

void PhotonMapper::storePhoton(Photon &p)
{
	photonMap.push_back(p);
}

void PhotonMapper::storeCausticPhoton(Photon &p)
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

Eigen::Vector3f PhotonMapper::computeRadiance(Eigen::Vector3f p, Interaction interaction)
{
	ANNpoint queryPoint;

	//gather photons around intersection point
	queryPoint = annAllocPt(3);
	for (int i = 0; i < 3; i++)
	{
		queryPoint[i] = p[i];
	}

	//1.query point 2.number of near neighbours,
	//3.nearest neighbours(returned) 4.distance(returned)
	//5. error bound
	kdTree->annkSearch(queryPoint, ESTIMATE, near_neighbour_idx, near_neighbour_dists, 0);

	// compute SUM( BRDF * PhotonPower)
	float maxDist = 0;
	float k = 1;
	Eigen::Vector3f sum(0,0,0);

	BRDF *brdf = (BRDF *)interaction.material;
	Eigen::Vector3f fr = brdf->eval(interaction);

	//find max distance
	for (int i = 0; i < ESTIMATE; i++)
	{
		if (near_neighbour_dists[i] > maxDist)
			maxDist = near_neighbour_dists[i];
	}

	for (int i = 0; i < ESTIMATE; i++)
	{
		Photon p = photonMap[near_neighbour_idx[i]];
		for (int j = 0; j < 3; j++)
			sum[j] += p.power[j] *  fr[j];
	}
	sum /= PI * maxDist * num;
	return sum;
}
