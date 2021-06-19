#pragma once

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <math.h>
#include <iostream>
#include "light.hpp"
#include "scene.hpp"
#include "PhotonMapper.h"
#include "utils.hpp"
#include "constant.hpp"
#include "ANN/ANN.h"

using namespace mathutils;

class PhotonTracer {
public:
	int nEmittedPhotons;

	PhotonTracer(Scene* scn,int nEmittedPhotons);
	void PhotonTracing(PhotonMapper map);

protected:
	Scene* scene;
	vector<Photon> m_photon;
};