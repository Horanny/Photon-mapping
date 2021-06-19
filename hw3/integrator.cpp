#ifndef NO_OMP
#include <omp.h>
#endif
#include "progressbar.hpp"
#include "integrator.hpp"
#include "constant.hpp"
#include "light.hpp"
#include "utils.hpp"
#include "config.hpp"
#include "brdf.hpp"

using namespace std;
using namespace mathutils;

extern Config conf;

/**
 * Integrator class
 */

Integrator::Integrator(Scene *scn, Camera *cam)
	: scene(scn), camera(cam)
{
}

/**
 * PathTracingIntegrator class
 */

PathTracingIntegrator::PathTracingIntegrator(Scene *scene, Camera *camera)
	: Integrator(scene, camera)
{
}

void PathTracingIntegrator::render()
{
	int dx, dy;
	int res_x = camera->getFilm().resolution.x(), res_y = camera->getFilm().resolution.y();

	/* Initialize a progress bar */
	progressbar progress_bar(res_x * res_y);
	int nEmittedPhotons = 100;

	PhotonMapper map;
	PhotonTracer tracer(scene, nEmittedPhotons);
	tracer.PhotonTracing(map);
	map.LoadToKDtree();

#ifndef NO_OMP
#pragma omp parallel for private(dy)
#endif
	for (dx = 0; dx < res_x; ++dx)
	{
		for (dy = 0; dy < res_y; ++dy)
		{
			/** TODO */
			//UNREACHABLE;
			int N = 1;
			vector<float> delx = unif(dx, dx + 1, N);
			vector<float> dely = unif(dy, dy + 1, N);

			Eigen::Vector3f totalRadiance(0, 0, 0);

			for (int i = 0; i < N; i++)
			{
				float deltax = delx[i];
				for (int j = 0; j < N; j++)
				{
					float deltay = dely[j];

					Ray ray = camera->generateRay(deltax, deltay);

					//if ray r hits the scene at p
					Interaction interaction;
					if (scene->intersection(ray, interaction))
					{
						//photonShade(ray, interaction, testmap);
						totalRadiance += radiance(ray, interaction, &map);
					}
				}
			}
			totalRadiance /= (N * N);

			camera->setPixel(dx, dy, totalRadiance);

#ifndef NO_OMP
#pragma omp critical
#endif
			progress_bar.update();
		}
	}
	annClose();
}

Eigen::Vector3f PathTracingIntegrator::radiance(Ray ray, Interaction interaction, PhotonMapper *map)
{
	/** TODO */
	//UNREACHABLE;
	if (interaction.type == Interaction::Type::LIGHT)
	{
		Light *light = scene->getLight();
		return light->emission(interaction.entry_point, -1 * ray.direction);
	}

	if (interaction.type == Interaction::Type::GEOMETRY)
	{
		interaction.wo = -1 * ray.direction;
		//return shadetest(ray, interaction);
		return photonShade(ray, interaction, map);
		//return Eigen::Vector3f(0, 0, 0);
	}
}


Eigen::Vector3f PathTracingIntegrator::shadetest(Ray ray, Interaction interaction)
{
	Eigen::Vector3f L_dir(0, 0, 0);
	Eigen::Vector3f L_indir(0, 0, 0);
	Eigen::Vector3f L(0, 0, 0);

	//Contribution from the light source
	Light *light = scene->getLight();
	IdealDiffusion *brdf = (IdealDiffusion *)interaction.material;
	Eigen::Vector3f fr = brdf->eval(interaction);

	//Uniformly sample the light at x' (pdf_light = 1/A)
	float *pdf;
	float pdf_light;

	Eigen::Vector3f samplepos = light->sample(interaction, pdf);
	pdf_light = light->samplePdf(interaction, samplepos);
	//Shoot a ray from p to x'
	Eigen::Vector3f p = interaction.entry_point;
	Eigen::Vector3f dir = (samplepos - p).normalized();
	interaction.wi = dir;
	Ray lightray(p, dir);
	//If the ray is not blocked in the middle
	if (!scene->isShadowed(lightray))
	{
		// L_dir = L_i * f_r * cos(theta) * cos(theta') / |x' - p | ^ 2 / pdf_light
		Eigen::Vector3f Li = light->emission(p, -1 * dir);

		float cos_theta = interaction.normal.dot(dir);
		Eigen::Vector3f light_n(0, -1, 0);
		float cos_theta_prime = light_n.dot(-1 * dir);
		float dist_p_light = (samplepos - p).norm();

		for (int j = 0; j < 3; j++)
		{
			L[j] = Li[j] * fr[j] * cos_theta * cos_theta_prime / (dist_p_light * dist_p_light) / pdf_light;
		}
	}

	//Contribution from reflection
	//Russian Roulette
	vector<float> ksivec = unif(0, 1, 1);
	float ksi = ksivec[0];
	float P_RR = 0.7;
	//Test Russian Roulette with probability P_RR
	if (ksi > P_RR)
	{
		L = Eigen::Vector3f(0, 0, 0);
		return L;
	}


	float hemipdf = brdf->sample(interaction);

	//trace a ray
	Ray hemiray(interaction.entry_point, interaction.wi);
	Interaction next_interact;

	if (scene->intersection(hemiray, next_interact))
	{
		//r hits an object
		if (next_interact.type == Interaction::Type::GEOMETRY)
		{
			next_interact.wo = -1 * hemiray.direction;
			Eigen::Vector3f tempL = shadetest(hemiray, next_interact);
			for (int i = 0; i < 3; i++)
			{
				float cos_theta = interaction.normal.dot(hemiray.direction);
				L[i] += tempL[i] * fr[i] * cos_theta / hemipdf / P_RR;
			}
		}
	}
	return L;
}

Eigen::Vector3f PathTracingIntegrator::photonShade(Ray ray, Interaction interaction, PhotonMapper *map)
{
	//generate ray from camera
	Eigen::Vector3f radiance(0, 0, 0);
	bool ISDIFFUSE = false;
	int count = 0;
	while (count < MAX_COUNT)
	{
		count += 1;
		//find the intersection point (in render)
		//if ray hits light source, compute light emission (in radiance)
		//generate BRDF based on the material of intersection point
		//compute distributions of reflection, and record reflect direction and reflect type
		//if reflect type is diffuse, then ISDIFFUSE = TRUE, break
		//else (reflect type is specular/transmission, compute next ray

		if (scene->intersection(ray, interaction) && interaction.type == Interaction::Type::GEOMETRY)
		{
			//generate the BRDF of the material
			//compute distribution of reflection, record reflect dir and reflect type
			BRDF *brdf = (BRDF *)interaction.material;
			Eigen::Vector3f fr = brdf->eval(interaction);
			float hemipdf = brdf->sample(interaction);

			//if reflect type is diffusion, ISDIFFUSE = TRUE, break
			if (interaction.material_type == Interaction::MaterialType::DIFFUSE)
			{
				ISDIFFUSE = true;
				break;
			}
			else
			{
				// ideal specular/mirror/transimite
				Ray hemiray(interaction.entry_point, interaction.wi);
				// Interaction next_interact;
				if (scene->intersection(hemiray, interaction))
				{
					if (interaction.type == Interaction::Type::LIGHT)
					{
						Light *light = scene->getLight();
						return light->emission(interaction.entry_point, -1 * hemiray.direction);

					}
					else if (interaction.type == Interaction::Type::GEOMETRY)
					{
						ray = hemiray;
						interaction.wo = -1 * hemiray.direction;
					}
				}
			}

			//if ISDIFFUSE = TRUE, then
			//1.find nearest n photons in kd tree
			//2.use density estimation
			//3.compute final radiance

			if (ISDIFFUSE) {
				radiance = map->computeRadiance(interaction.entry_point,interaction);
			}
		}
	}
	return radiance;
}