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

Integrator::Integrator(Scene* scn, Camera* cam)
    : scene(scn)
    , camera(cam)
{
}


/**
 * PathTracingIntegrator class
 */

PathTracingIntegrator::PathTracingIntegrator(Scene* scene, Camera* camera)
    : Integrator(scene, camera)
{  
}

void PathTracingIntegrator::render()
{
    int dx, dy;
    int res_x = camera->getFilm().resolution.x(), res_y = camera->getFilm().resolution.y();

    /* Initialize a progress bar */
    progressbar progress_bar(res_x * res_y);

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
			vector<float> delx = unif(dx, dx+1, N);
			vector<float> dely = unif(dy, dy+1, N);

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
						totalRadiance += radiance(ray, interaction);
					}
				}
			}
			totalRadiance /= (N*N);

			camera->setPixel(dx, dy, totalRadiance);

#ifndef NO_OMP
            #pragma omp critical
#endif
            progress_bar.update();
        }
    }
}

Eigen::Vector3f PathTracingIntegrator::radiance(Ray ray, Interaction interaction)
{
    /** TODO */
    //UNREACHABLE;
	if (interaction.type == Interaction::Type::LIGHT)
	{
		Light* light = scene->getLight();
		return light->emission(interaction.entry_point, -1 * ray.direction);
	}

	if (interaction.type == Interaction::Type::GEOMETRY)
	{
		interaction.wo = -1 * ray.direction;
		//return shade(ray, interaction);
		return shadetest(ray, interaction);
	}
}

Eigen::Vector3f PathTracingIntegrator::shade(Ray ray, Interaction interaction)
{
	Eigen::Vector3f L_dir(0, 0, 0);
	Eigen::Vector3f L_indir(0, 0, 0);

	#pragma region Contribution from the light source
	Light* light = scene->getLight();
	IdealDiffusion* brdf = (IdealDiffusion*)interaction.material;

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
		Eigen::Vector3f fr = brdf->eval(interaction);
		float cos_theta = interaction.normal.dot(dir);
		Eigen::Vector3f light_n(0, -1, 0);
		float cos_theta_prime = light_n.dot(-1*dir);
		float dist_p_light = (samplepos - p).norm();

		for (int j = 0; j < 3; j++)
		{
			L_dir[j] =  Li[j] * fr[j] * cos_theta * cos_theta_prime / (dist_p_light * dist_p_light) / pdf_light;
		}
	}
	
	#pragma endregion

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

	//	//Uniformly sample the hemisphere toward wi
	//	float pdf_hemi = in_brdf->sample(interaction);

	//	//Trace a ray r(p, wi)
	//	Eigen::Vector3f in_dir = interaction.wi;
	//	Eigen::Vector3f in_p = interaction.entry_point;
	//	float in_cos_theta = interaction.normal.dot(in_dir);
	//	if (in_cos_theta < 0)		in_cos_theta *= -1;
	//	//cout << "costheta " << cos_theta << endl;

	//	Ray hemiray(in_p, in_dir);
	//	Interaction next_interaction;

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

	return L_dir + L_indir;

}

Eigen::Vector3f PathTracingIntegrator::shadetest(Ray ray, Interaction interaction)
{
	Eigen::Vector3f L_dir(0, 0, 0);
	Eigen::Vector3f L_indir(0, 0, 0);
	Eigen::Vector3f L(0, 0, 0);

	//Contribution from the light source
	Light* light = scene->getLight();
	IdealDiffusion* brdf = (IdealDiffusion*)interaction.material;
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
	
	//随机生成1个方向wi并以pdf（w)分布
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
