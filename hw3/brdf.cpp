#include <vector>
#include "Eigen/Dense"
#include "interaction.hpp"
#include "utils.hpp"
#include "constant.hpp"
#include "brdf.hpp"
#include <time.h>
using namespace std;
using namespace mathutils;

/**
 * BRDF class
 */

BRDF::BRDF()
{
}


/**
 * IdealDiffusion class
 */

IdealDiffusion::IdealDiffusion(Eigen::Vector3f diff)
    : reflectivity(diff),type(BRDF::Type::DIFFUSE)
{
}
    
Eigen::Vector3f IdealDiffusion::eval(const Interaction& interact)
{
    /** TODO */
    //UNREACHABLE;
	return reflectivity;
}

float IdealDiffusion::samplePdf(const Interaction& interact)
{
    /** TODO */
    //UNREACHABLE;
	Eigen::Vector3f wi = interact.wi;
	Eigen::Vector3f n = interact.normal;
	float pdf;
	pdf = n.dot(wi) / PI;
	//float pdf = 1 / (2 * PI);

	return pdf;
}

float IdealDiffusion::sample(Interaction& interact)
{
    /** TODO */
    //UNREACHABLE

	//concentric mapping
	Eigen::Vector2f sample_disk = ConcentricSampleDisk();
	float z = sqrt(max(float(0), 1 - sample_disk[0] * sample_disk[0] - sample_disk[1] * sample_disk[1]));
	Eigen::Vector3f newsample(sample_disk[0], sample_disk[1], z);

	//find transformation matrix
	Eigen::Vector3f p = interact.entry_point;
	Eigen::Vector3f n = interact.normal;

	Eigen::Matrix3f rot;
	rot = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 0, 1), n).toRotationMatrix();

	//newsample = rot * newsample;
	newsample = rot * newsample;
	
	interact.wi = newsample.normalized();
	
	//uniform samplesphere
	//vector<float> u0 = unif(0, 1, 1);
	//vector<float> u1 = unif(0, 1, 1);
	//float z = u0[0];
	//float r = sqrt(max(0.0, 1.0 - z * z));
	//float phi = 2 * PI * u1[0];
	//Eigen::Vector3f newsample(r * cos(phi), r * sin(phi), z);
	//Eigen::Vector3f normal(0, 0, 1);
	//
	////find transformation matrix
	//Eigen::Vector3f p = interact.entry_point;
	//Eigen::Vector3f n = interact.normal;
	//
	//Eigen::Matrix3f rot;
	//rot = Eigen::Quaternionf::FromTwoVectors(normal, n).toRotationMatrix();
	//
	//interact.wi = newsample.normalized();

	//get pdf
	float pdf;
	pdf = samplePdf(interact);

	return pdf;
}

bool IdealDiffusion::isDelta() const
{
    return false;
}


/**
 * IdealSpecular class
 */

IdealSpecular::IdealSpecular(Eigen::Vector3f spec)
    : reflectivity(spec),type(BRDF::Type::SPECULAR)
{
}

Eigen::Vector3f IdealSpecular::eval(const Interaction& interact)
{
    /** TODO */
    //UNREACHABLE;
	return reflectivity;
}

float IdealSpecular::samplePdf(const Interaction& interact)
{
    /** TODO */
	return 0.0;
}

float IdealSpecular::sample(Interaction& interact)
{
    /** TODO */
	return 0.0;
}

bool IdealSpecular::isDelta() const
{
    return true;
}


/**
 * IdealTransmission class
 */

IdealTransmission::IdealTransmission(Eigen::Vector3f reflect, float idx_refract)
    : reflectivity(reflect)
    , ior(idx_refract),
	type(BRDF::Type::TRANSMISSION)
{

}

Eigen::Vector3f IdealTransmission::eval(const Interaction& interact)
{
    /** TODO */
	return Eigen::Vector3f(0, 0, 0);
}

float IdealTransmission::samplePdf(const Interaction& interact)
{
    /** TODO */
	Eigen::Vector3f wi = interact.wi;
	Eigen::Vector3f n = interact.normal;
	float pdf;
	pdf = -n.dot(wi) / PI;

	return pdf;
}

float IdealTransmission::sample(Interaction& interact)
{
    /** TODO */
	Eigen::Vector2f sample_disk = ConcentricSampleDisk();
	float z = sqrt(max(float(0), 1 - sample_disk[0] * sample_disk[0] - sample_disk[1] * sample_disk[1]));
	Eigen::Vector3f newsample(sample_disk[0], sample_disk[1], z);

	//find transformation matrix
	Eigen::Vector3f p = interact.entry_point;
	Eigen::Vector3f n = interact.normal;

	Eigen::Matrix3f rot;
	rot = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 0, 1), n).toRotationMatrix();

	//newsample = rot * newsample;
	newsample = rot * newsample;
	
	interact.wi = -newsample.normalized();

	float pdf;
	pdf = samplePdf(interact);

	return pdf;
}

bool IdealTransmission::isDelta() const
{
    return true;
}
