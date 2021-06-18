#include <utility>
#include "light.hpp"
#include "geometry.hpp"
#include "utils.hpp"
using namespace mathutils;

/**
 * Light class
 */

Light::Light(Eigen::Vector3f pos, Eigen::Vector3f power)
    : position(pos)
    , radiance(power)
{
}

Eigen::Vector3f Light::getPosition() const
{
    return position;
}

Eigen::Vector3f Light::getRadiance() const
{
    return radiance;
}

Ray Light::generateRay(Eigen::Vector3f sample)
{
	//concentric mapping
	Eigen::Vector2f sample_disk = ConcentricSampleDisk();
	float z = sqrt(max(float(0), 1 - sample_disk[0] * sample_disk[0] - sample_disk[1] * sample_disk[1]));
	Eigen::Vector3f newsample(sample_disk[0], sample_disk[1], z);

	//find transformation matrix
	Eigen::Vector3f n(0, -1, 0);
	Eigen::Vector3f p = sample;

	Eigen::Matrix3f rot;
	rot = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 0, 1), n).toRotationMatrix();

	//newsample = rot * newsample;
	newsample = rot * newsample;

	return Ray(p, newsample);
}

Eigen::Vector3f Light::SimplySample()
{
	return Eigen::Vector3f();
}


/**
 * AreaLight class
 */
AreaLight::AreaLight(Eigen::Vector3f pos, Eigen::Vector3f power, Eigen::Vector2f size)
    : Light(pos, power)
    , area_size(size)
    , geometry_delegation(
        pos - Eigen::Vector3f(size[0], 0, size[1]) / 2,
        Eigen::Vector3f(size[0], 0, 0),
        Eigen::Vector3f(0, 0, size[1]),
        Eigen::Vector3f(0, 1, 0), nullptr)
{
}

Eigen::Vector3f AreaLight::emission(Eigen::Vector3f pos, Eigen::Vector3f dir)
{
    /** TODO */
    //UNREACHABLE;
	/* Get the emission at the specified position along the given direction */

	Eigen::Vector3f n(0, -1, 0);
	float cosine = fabs(n.dot(dir));

	return cosine * radiance;
	//return radiance;
}

float AreaLight::samplePdf(const Interaction& ref_it, Eigen::Vector3f pos)
{
    /** TODO */
	/* Compute the PDF of the given light sample */
	float pdf;
	float Area = area_size[0] * area_size[1];
	pdf = 1 / Area;

	return pdf;
}

Eigen::Vector3f AreaLight::sample(Interaction& ref_it, float* pdf)
{
    /** TODO */
    //UNREACHABLE;
	/* Sample a position on the light and obtain the corresponding PDF */
	Eigen::Vector3f sample(0, 0, 0);
	Eigen::Vector3f pos;
	pos = position - Eigen::Vector3f(area_size[0], area_size[1], 0) / 2;
	vector<float> sam_x;
	vector<float> sam_y;
	sam_x = unif(pos[0], pos[0] + area_size[0], 1);
	sam_y = unif(pos[1], pos[1] + area_size[1], 1);
	sample[0] = sam_x[0];
	sample[1] = sam_y[0];
	sample[2] = pos[2];

	sam_x.clear();
	sam_y.clear();

	return sample;
}

bool AreaLight::rayIntersection(Interaction& interaction, const Ray& ray)
{
    bool intersection = geometry_delegation.rayIntersection(interaction, ray);
    interaction.type = Interaction::Type::LIGHT;
    return intersection;
}

Eigen::Vector3f AreaLight::SimplySample()
{
	Eigen::Vector3f sample(0, 0, 0);
	Eigen::Vector3f pos;
	pos = position - Eigen::Vector3f(area_size[0], area_size[1], 0) / 2;
	vector<float> sam_x;
	vector<float> sam_y;
	sam_x = unif(pos[0], pos[0] + area_size[0], 1);
	sam_y = unif(pos[1], pos[1] + area_size[1], 1);
	sample[0] = sam_x[0];
	sample[1] = sam_y[0];
	sample[2] = pos[2];

	sam_x.clear();
	sam_y.clear();

	return sample;
}

//get pdfA(x)*pdfw(w)
float AreaLight::totalpdf(Eigen::Vector3f dir)
{
	float pdf;
	float Area = area_size[0] * area_size[1];
	pdf = 1 / Area;

	Eigen::Vector3f n(0, -1, 0);
	pdf *= n.dot(dir) / PI;

	return pdf;
}

