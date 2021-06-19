#include "PhotonTracer.h"

PhotonTracer::PhotonTracer(Scene* scn, int nEmittedPhotons)
	:scene(scn),nEmittedPhotons(nEmittedPhotons)
{
}

void PhotonTracer::PhotonTracing(PhotonMapper map)
{

	for (int i = 0; i < nEmittedPhotons; i++)
	{	
		//sample the light, get the ray and initial power
		Light *light = scene->getLight();

		Eigen::Vector3f sample = light->SimplySample();
		Ray ray = light->generateRay(sample);
		//initial power
		Eigen::Vector3f power = light->emission(sample, ray.direction);
		power /= light->totalpdf(ray.direction);
		
		int count = 0;
		while (count < MAX_COUNT)
		{
			Interaction interaction;

			//find the interaction of the ray and the scene
			if (scene->intersection(ray, interaction) && interaction.type == Interaction::Type::GEOMETRY)
			{
				//generate the BRDF of the interaction			
				//compute distribution of reflection, record reflect dir and reflect type
				BRDF *brdf = (BRDF*)interaction.material;
				Eigen::Vector3f fr = brdf->eval(interaction);
				float hemipdf = brdf->sample(interaction);

				//if (brdf->type == BRDF::Type::DIFFUSE)
				//	cout << "diffuse~" << endl;

				//if reflect type is diffusion, put it to photon map
				if (interaction.material_type == Interaction::MaterialType::DIFFUSE)
				{
					Photon p;
					p.pos = interaction.entry_point;
					p.dir = interaction.wi;
					p.power = power;
					map.storePhoton(p);
				}
				//else if (interaction.type == Interaction::MaterialType::SPECULAR)
				//else if (interaction.type == Interaction::MaterialType::TRANSMISSION)

				//use Russian Roulette to know whether the photon is absorbed
				vector<float> ksivec = unif(0, 1, 1);
				float ksi = ksivec[0];
				float P_RR = 0.7;
				if (ksi > P_RR)
				{
					break;
				}

				//if not absorbed, update power
				for (int i = 0; i < 3; i++)
				{
					power[i] *= fr[i];
				}

				//update the new ray by reflect dir
				ray.origin = interaction.entry_point;
				ray.direction = interaction.wi;

			}
			else
				break;
		}	
	}
}

