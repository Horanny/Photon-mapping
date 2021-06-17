#pragma once
#include "scene.hpp"
#include "camera.hpp"
#include "interaction.hpp"


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
};	

//Eigen::Vector3f PathTracingIntegrator::shade(Ray ray, Interaction interaction)
//{
//	Eigen::Vector3f L_dir(0, 0, 0);
//	Eigen::Vector3f L_indir(0, 0, 0);
//
//#pragma region Contribution from the light source
//	Light* light = scene->getLight();
//	IdealDiffusion* brdf = (IdealDiffusion*)interaction.material;
//
//	//Uniformly sample the light at x' (pdf_light = 1/A)
//	float *pdf;
//	float pdf_light;
//
//	Eigen::Vector3f samplepos = light->sample(interaction, pdf);
//	pdf_light = light->samplePdf(interaction, samplepos);
//	//Shoot a ray from p to x'
//	Eigen::Vector3f p = interaction.entry_point;
//	Eigen::Vector3f dir = (samplepos - p).normalized();
//	interaction.wi = dir;
//	Ray lightray(p, dir);
//	//If the ray is not blocked in the middle
//	if (!scene->isShadowed(lightray))
//	{
//		// L_dir = L_i * f_r * cos(theta) * cos(theta') / |x' - p | ^ 2 / pdf_light
//		Eigen::Vector3f Li = light->emission(p, -1 * dir);
//		Eigen::Vector3f fr = brdf->eval(interaction);
//		float cos_theta = interaction.normal.dot(dir);
//		Eigen::Vector3f light_n(0, -1, 0);
//		float cos_theta_prime = light_n.dot(-1 * dir);
//		float dist_p_light = (samplepos - p).norm();
//
//		for (int j = 0; j < 3; j++)
//		{
//			L_dir[j] = Li[j] * fr[j] * cos_theta * cos_theta_prime / (dist_p_light * dist_p_light) / pdf_light;
//		}
//	}
//
//#pragma endregion
//
//#pragma region Contribution from other reflectors
//vector<float> ksivec = unif(0, 1, 1);
//float ksi = ksivec[0];
//float P_RR = 0.5;
////Test Russian Roulette with probability P_RR
//if (ksi > P_RR)
//{
//	L_indir = Eigen::Vector3f(0, 0, 0);
//}
//else
//{
//	IdealDiffusion* in_brdf = (IdealDiffusion*)interaction.material;
//	Eigen::Vector3f in_fr = in_brdf->eval(interaction);
//
//	//Uniformly sample the hemisphere toward wi
//	float pdf_hemi = in_brdf->sample(interaction);
//
//	//Trace a ray r(p, wi)
//	Eigen::Vector3f in_dir = interaction.wi;
//	Eigen::Vector3f in_p = interaction.entry_point;
//	float in_cos_theta = interaction.normal.dot(in_dir);
//	if (in_cos_theta < 0)		in_cos_theta *= -1;
//	//cout << "costheta " << cos_theta << endl;
//
//	Ray hemiray(in_p, in_dir);
//	Interaction next_interaction;
//
//	//If ray r hit a non-emitting object at q
//	if (scene->intersection(hemiray, next_interaction))
//	{
//		if (next_interaction.type == Interaction::Type::GEOMETRY)
//		{
//			next_interaction.wo = -1 * hemiray.direction;
//			for (int j = 0; j < 3; j++)
//			{
//				//	L_indir = shade(q, -wi) * f_r * cos(theta) / pdf_hemi / P_RR
//				//shade(hemiray, next_interaction);
//				L_indir[j] = shade(hemiray, next_interaction)[j] * in_fr[j] * in_cos_theta / pdf_hemi / P_RR;
//			}
//		}
//	}
//}
//#pragma endregion
//
//	return L_dir + L_indir;
//
//}
