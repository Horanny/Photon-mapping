 #pragma once
#include "scene.hpp"
#include "camera.hpp"
#include "interaction.hpp"
#include "PhotonTracer.h"

/**
 * Base class of integrator
 */

class Integrator
{
protected:
	Scene* scene;
	Camera* camera;

public:
	Integrator(Scene* scn, Camera* cam);
	virtual ~Integrator() = default;
	
	virtual void render() = 0;
	virtual Eigen::Vector3f radiance(Ray ray, Interaction interaction) = 0;
};


/**
 * Path-tracing integrator
 */

class PathTracingIntegrator : public Integrator
{
public:
    PathTracingIntegrator(Scene* scene, Camera* camera);
    virtual void render() override;
    virtual Eigen::Vector3f radiance(Ray ray, Interaction interaction) override;
	Eigen::Vector3f shade(Ray ray, Interaction interaction);
	Eigen::Vector3f shadetest(Ray ray, Interaction interaction);
	
	Eigen::Vector3f photonShade(Ray ray, Interaction interaction);
};	

