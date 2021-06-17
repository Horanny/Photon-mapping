#include <random>
#include <algorithm>
#include "utils.hpp"
#include "constant.hpp"
#include  <ctype.h>
#include <random>
#include <algorithm>

/**
 * strutils namespace
 */

void strutils::ltrim(std::string& s)
{
   s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(isspace))));
}

void strutils::rtrim(std::string& s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(isspace))).base(), s.end());
}

void strutils::trim(std::string& s)
{
    ltrim(s);
    rtrim(s);
}


/**
 * mathutils namespace
 */


float mathutils::clamp(float x, float lo, float hi)
{
    return x < lo ? lo : x > hi ? hi : x;
}

unsigned char mathutils::gamma_correction(float x)
{
	return (unsigned char)(pow(mathutils::clamp(x, 0.0, 1.0), 1 / 2.2) * 255);
}

static std::default_random_engine random_generator;

std::vector<float> mathutils::unif(float a, float b, int N)
{
    std::vector<float> res;
    std::uniform_real_distribution<float> dis(a, b);

    for (int i = 0; i < N; i++) {
        res.push_back(dis(random_generator));
    }
    return res;
}

Eigen::Vector2f mathutils::ConcentricSampleDisk()
{
		//1.map uniform random numbers to [-1,1]^2
		Eigen::Vector2f u;
		std::vector<float> u0 = unif(-1, 1, 1);
		std::vector<float> u1 = unif(-1, 1, 1);
		u[0] = u0[0];
		u[1] = u1[0];

		//2.handle degeneracy at the origin
		if (u[0] == 0 && u[1] == 0)
			return u;

		//3.apply concentric mapping to point
		float theta, r;
		if (abs(u[0]) > abs(u[1]))
		{
			r = u[0];
			theta = PI / 4 * (u[1] / u[0]);
		}
		else
		{
			r = u[1];
			theta = PI / 2 - PI / 4 * (u[0] / u[1]);
		}
		return Eigen::Vector2f(r*cos(theta), r*sin(theta));
	

}

float mathutils::random_select(float a, float b)
{
	std::default_random_engine random_generator;
	std::uniform_real_distribution<float> dis(a, b);
	float random_val = dis(random_generator);

	return random_val;
}

