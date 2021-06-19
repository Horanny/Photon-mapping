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
	Vec3<T> pHit; // point of intersection
	Vec3<T> nHit; // normal at the intersection point
	GeomObject<T> *obj = NULL;
	if (!checkIntersection(rayOrig, rayDir, obj, pHit, nHit)) // closest object
		return Vec3<T>(0);

	Vec3<T> surfaceColor = 0; // color of the ray/surfaceof the object intersected by the ray

	nHit.normalize(); // normalize normal direction

	// If the normal and the view direction are not opposite to each other
	// reverse the normal direction. That also means we are inside the sphere so set
	// the inside bool to true. Finally reverse the sign of IdotN which we want
	// positive.
	T bias = 1e-4; // add some bias to the point from which we will be tracing
	bool inside = false;
	if (rayDir.dot(nHit) > 0)
		nHit = -nHit, inside = true;
	if ((obj->transparency > 0 || obj->reflection > 0) && depth < MAX_RAY_DEPTH)
	{
		T facingratio = -rayDir.dot(nHit);
		// change the mix value to tweak the effect
		T fresneleffect = mix<T>(pow(1 - facingratio, 3), 1, 0.6);
		// compute reflection direction (not need to normalize because all vectors
		// are already normalized)
		Vec3<T> refldir = rayDir - nHit * 2 * rayDir.dot(nHit);
		refldir.normalize();
		Vec3<T> reflection = trace(pHit + nHit * bias, refldir, depth + 1);

		Vec3<T> refraction = 0;
		// if the sphere is also transparent compute refraction ray (transmission)
		if (obj->transparency)
		{
			T ior = 1.1, eta = (inside) ? ior : 1 / ior; // are we inside or outside the surface?
			T cosi = -nHit.dot(rayDir);
			T k = 1 - eta * eta * (1 - cosi * cosi);
			Vec3<T> refrdir = rayDir * eta + nHit * (eta * cosi - sqrt(k));
			refrdir.normalize();
			refraction = trace(pHit - nHit * bias, refrdir, depth + 1);
		}
		// the result is a mix of reflection and refraction (if the sphere is transparent)
		surfaceColor = (reflection * fresneleffect * obj->reflection +
						refraction * (1) * obj->transparency) *
					   obj->surfaceColor;
	}
	else
		surfaceColor = computeRadiance(pHit, obj);

	return surfaceColor;
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
	Eigen::Vector3f sum;

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
		float w = 1 - near_neighbour_dists[i] / (k * maxDist);
		for (int j = 0; j < 3; j++)
			sum[i] += photonMap[near_neighbour_idx[i]].power[i] * w * fr[i];
	}
	return sum;
}
