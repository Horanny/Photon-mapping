#include <math.h>
#include "geometry.hpp"
#include "constant.hpp"
#include "utils.hpp"
#include "objloader.hpp"
using namespace std;

/**
 * Geometry class
 */

void Geometry::setMaterial(BRDF* new_mat)
{
    material = new_mat;
}


/**
 * Parallelogram class
 */

Parallelogram::Parallelogram(Eigen::Vector3f p0, Eigen::Vector3f s0, Eigen::Vector3f s1, Eigen::Vector3f normal, BRDF* mat)
    : p0(p0)
    , normal(normal.normalized())
{
    s0_len = s0.norm();
    s1_len = s1.norm();
    this->s0 = s0.normalized();
    this->s1 = s1.normalized();

    setMaterial(mat);
    buildBoundingBox();
}

bool Parallelogram::rayIntersection(Interaction& interaction, const Ray& ray)
{
    if (ray.direction.dot(normal) == 0)
    {
        return false;
    }
    
    float t = (p0 - ray.origin).dot(normal) / ray.direction.dot(normal);
    Eigen::Vector3f p0_p = ray.getPoint(t) - p0;
    float q0 = p0_p.dot(s0) / s0_len;
    float q1 = p0_p.dot(s1) / s1_len;
    if (q0 >= 0 && q0 <= s0.norm() && q1 >= 0 && q1 <= s1.norm() && t >= ray.range_min && t <= ray.range_max)
    {
        interaction.entry_dist = t;
        interaction.exit_dist = t;
        interaction.normal = normal;
        interaction.entry_point = ray.getPoint(t);
        if (material != nullptr)
        {
            interaction.material = (void*)material;
			
			if (material->type == BRDF::Type::DIFFUSE)
			{
				//cout << "diffuse!" << endl;
				interaction.material_type = Interaction::MaterialType::DIFFUSE;
			}

			else if(material->type == BRDF::Type::SPECULAR)
			interaction.material_type = Interaction::MaterialType::SPECULAR;

			else if (material->type == BRDF::Type::TRANSMISSION)
			interaction.material_type = Interaction::MaterialType::TRANSMISSION;

        }
        interaction.type = Interaction::Type::GEOMETRY;


        return true;
    }
    return false;
}

void Parallelogram::buildBoundingBox()
{
    Eigen::Vector3f p1 = p0 + s0 + s1;
    bounding_box.lb = p0.cwiseMin(p1);
    bounding_box.ub = p0.cwiseMax(p1);
}


/**
 * Sphere class
 */

Sphere::Sphere(Eigen::Vector3f p0, float r, BRDF* mat)
    : p0(p0)
    , radius(r)
{
    setMaterial(mat);
    buildBoundingBox();
}

bool Sphere::rayIntersection(Interaction& interaction, const Ray& ray)
{
    float a = 1.0f;
    float b = 2 * ray.direction.dot(ray.origin - p0);
    float c = (ray.origin - p0).squaredNorm() - radius * radius;
    float delta = b * b - 4 * a * c;

    if (delta < 0)
    {
        return false;
    }

    float t0 = (-b - sqrt(delta)) / 2 * a;
    float t1 = (-b + sqrt(delta)) / 2 * a;

    if (t1 < 0)
    {
        return false;
    }
    else if (t0 < 0 && t1 >= 0)
    {
        t0 = t1;
    }

    if (t0 < ray.range_min || t0 > ray.range_max)
    {
        return false;
    }

    interaction.entry_dist = t0;
    interaction.exit_dist = t1;
    interaction.entry_point = ray.getPoint(t0);
    Eigen::Vector3f r_vec = interaction.entry_point - p0;
    interaction.normal = r_vec.normalized();
    if (material != nullptr)
    {
        interaction.material = (void*)material;
		if (material->type == BRDF::Type::DIFFUSE)
		{
			//cout << "diffuse!" << endl;
			interaction.material_type = Interaction::MaterialType::DIFFUSE;
		}

		else if (material->type == BRDF::Type::SPECULAR)
			interaction.material_type = Interaction::MaterialType::SPECULAR;

		else if (material->type == BRDF::Type::TRANSMISSION)
			interaction.material_type = Interaction::MaterialType::TRANSMISSION;

    }
    interaction.type = Interaction::Type::GEOMETRY;


    return true;
}

void Sphere::buildBoundingBox()
{
    bounding_box = AABB(p0, radius);
}


/**
 * TriangleMesh class
 */

TriangleMesh::TriangleMesh(std::string file_path, BRDF* mat)
{
    setMaterial(mat);

    std::vector<Eigen::Vector2f> out_uvs;
    std::vector<int> out_vt_index;
    loadObj(file_path.c_str(), vertices, out_uvs, normals, vertices_index, out_vt_index, normals_index);
    
    num_triangles = vertices_index.size() / 3;

    has_grid = false;

    buildBoundingBox();
}

bool TriangleMesh::raySingleTriangleIntersection(Interaction& interaction, const Ray& ray, int v0_idx, int v1_idx, int v2_idx) const
{
    /**
     * TODO: Ray intersection test with single triangle
     * Note: Remember that normals are interpolated using barycentric coordinates.
     */
	//Cramer's Rule to solve t, u, v
	Eigen::Vector3f ori = ray.origin;
	Eigen::Vector3f dir = ray.direction;
	Eigen::Vector3f v0 = vertices[v0_idx];
	Eigen::Vector3f v1 = vertices[v1_idx];
	Eigen::Vector3f v2 = vertices[v2_idx];

	double u, v, t;

	Eigen::Vector3f E1 = v1 - v0;
	Eigen::Vector3f E2 = v2 - v0;
	Eigen::Vector3f T = ori - v0;
	//det = (E1xE2).dot(D)
	float det = (E2.cross(E1)).dot(dir);

	//ray and triangle are parallel if det is close to 0
	if (fabs(det) < 0.001)
		return false;

	//u = |-D T E2|/det
	u = (E2.cross(T)).dot(dir);
	u /= det;
	if (u < 0 || u > 1)
		return false;

	//v = |-D E1 T|/det
	v = (T.cross(E1)).dot(dir);
	v /= det;
	if (v < 0 || v>1 || u + v > 1)
	{
		return false;
	}

	//t = |T E1 E2|/det
	t = -1 * (E2.cross(E1)).dot(T);
	t /= det;

	interaction.entry_dist = t;
	interaction.exit_dist = t;
	interaction.entry_point = ray.getPoint(t);
	interaction.normal = (E1.cross(E2)).normalized();

	if (material != nullptr)
	{
		interaction.material = (void*)material;

		if (material->type == BRDF::Type::DIFFUSE)
		{
			//cout << "diffuse!" << endl;
			interaction.material_type = Interaction::MaterialType::DIFFUSE;
		}

		else if (material->type == BRDF::Type::SPECULAR)
			interaction.material_type = Interaction::MaterialType::SPECULAR;

		else if (material->type == BRDF::Type::TRANSMISSION)
			interaction.material_type = Interaction::MaterialType::TRANSMISSION;

	}
	interaction.type = Interaction::Type::GEOMETRY;

	return true; 
}

bool TriangleMesh::rayIntersection(Interaction& interaction, const Ray& ray)
{
    Interaction final_interaction;
    if (has_grid) {
        /**
         * TODO: Use uniform grid to handle triangle intersection here
         * Note: Grid traversal algorithm must be used here.
         */
		 //we need to first find the intersection point of the aabb and the ray

		float tin, tout;
		if (bounding_box.rayIntersection(ray,tin,tout))
		{
			Eigen::Vector3f dir = ray.direction;
			Eigen::Vector3f ori = ray.origin + tin * dir;
			Eigen::Vector3f lb = bounding_box.lb;
			Eigen::Vector3f ub = bounding_box.ub;
			Eigen::Vector3f delta(0, 0, 0);
			float cellsize = bounding_box.cellsize;

			#pragma region get resolution
			float xlength, ylength, zlength;
			float resol_x, resol_y, resol_z;

			xlength = bounding_box.getDist(0);
			ylength = bounding_box.getDist(1);
			zlength = bounding_box.getDist(2);

			resol_x = xlength / cellsize;
			resol_y = ylength / cellsize;
			resol_z = zlength / cellsize;
			#pragma endregion

			//compute tx0,ty0 and tz0
			float tx, ty, tz;
			if (dir[0] < 0)
			{
				tx = floor((ori[0] - lb[0]) / cellsize);
				tx = (tx * cellsize - (ori[0] - lb[0])) / dir[0];
				delta[0] = -1 * (cellsize / dir[0]);
			}
			else
			{
				tx = floor((ori[0] - lb[0]) / cellsize) + 1;
				tx = (tx * cellsize - (ori[0] - lb[0])) / dir[0];
				delta[0] = cellsize / dir[0];
			}

			if (dir[1] < 0)
			{
				ty = floor((ori[1] - lb[1]) / cellsize);
				ty = (ty * cellsize - (ori[1] - lb[1])) / dir[1];
				delta[1] = -1 * (cellsize / dir[1]);
			}
			else
			{
				ty = floor((ori[1] - lb[1]) / cellsize) + 1;
				ty = (ty * cellsize - (ori[1] - lb[1])) / dir[1];
				delta[1] = cellsize / dir[1];
			}

			if (dir[2] < 0)
			{
				tz = floor((ori[2] - lb[2]) / cellsize);
				tz = (tz * cellsize - (ori[2] - lb[2])) / dir[2];
				delta[2] = -1 * (cellsize / dir[2]);
			}
			else
			{
				tz = floor((ori[2] - lb[2]) / cellsize) + 1;
				tz = (tz * cellsize - (ori[2] - lb[2])) / dir[2];
				delta[2] = cellsize / dir[2];
			}

			//DDA alg
			float t = 0;
			int x, y, z;//origin of the ray
			x = floor((ori[0] - lb[0]) / cellsize);
			y = floor((ori[1] - lb[1]) / cellsize);
			z = floor((ori[2] - lb[2]) / cellsize);
			if (x == resol_x)	x -= 1;
			if (y == resol_y)	y -= 1;
			if (z == resol_z)	z -= 1;
			if (x == -1)	x += 1;
			if (y == -1)	y += 1;
			if (z == -1)	z += 1;

			while (1)
			{	
				vector<int> vertindex = bounding_box.gridindex[x][y][z].tri_index;
				//cout << vertindex.size() << endl;
				for (int i = 0; i < vertindex.size(); i++)
				{
					if (vertindex.size() == 0)
						break;
					else
					{
						int temp = i;
						//cout << "size: "<<vertindex.size() << endl;
						//cout << "i: "<<i << endl;
						int v1index = vertices_index[vertindex[temp]];
						int v2index = vertices_index[vertindex[temp] + 1];
						int v3index = vertices_index[vertindex[temp] + 2];
						Interaction cur;
						if (raySingleTriangleIntersection(cur, ray, v1index, v2index, v3index))
						{
							//if t <= max allowed, it's in the current voxel
							//otherwise, continue traversal until either we reach the voxel 
							//where the intersections occurrs
							//or find a closer object
							if (final_interaction.entry_dist == -1 || cur.entry_dist < final_interaction.entry_dist)
								final_interaction = cur;
						}
					}

				}

				if (tx < ty)
				{
					if (tx < tz)
					{
						t = tx;
						tx += delta[0];
						if (dir[0] < 0)
							x -= 1;
						else
							x += 1;
					}
					else
					{
						t = tz;
						tz += delta[2];
						if (dir[2] < 0)
							z -= 1;
						else
							z += 1;
					}
				}
				else //tx>ty
				{
					if (ty < tz)
					{
						t = ty;
						ty += delta[1];
						if (dir[1] < 0)
							y -= 1;
						else
							y += 1;
					}
					else
					{
						t = tz;
						tz += delta[2];
						if (dir[2] < 0)
							z -= 1;
						else
							z += 1;
					}
				}
				//break condition
				if (final_interaction.entry_dist < t && final_interaction.entry_dist != -1)
					break;

				if (x < 0 || y < 0 || z < 0 ||
					x >= resol_x || y >= resol_y || z >= resol_z)
					break;

			}
		}

    } else {
        for (int i = 0; i < vertices_index.size(); i+=3) {
            Interaction cur_it;
			int v1index = vertices_index[i];
			int v2index = vertices_index[i + 1];
			int v3index = vertices_index[i + 2];
            if (raySingleTriangleIntersection(cur_it, ray, v1index, v2index, v3index)) {
                if (final_interaction.entry_dist == -1 || cur_it.entry_dist < final_interaction.entry_dist) {
                    final_interaction = cur_it;
                }
            }
        }
    }

    if (final_interaction.entry_dist != -1)
    {
        interaction = final_interaction;
        if (material != nullptr)
        {
            interaction.material = material;
			if (material->type == BRDF::Type::DIFFUSE)
				interaction.material_type = Interaction::MaterialType::DIFFUSE;

			else if (material->type == BRDF::Type::SPECULAR)
				interaction.material_type = Interaction::MaterialType::SPECULAR;

			else if (material->type == BRDF::Type::TRANSMISSION)
				interaction.material_type = Interaction::MaterialType::TRANSMISSION;

        }
        interaction.type = Interaction::Type::GEOMETRY;

        return true;
    }

    return false;
}

void TriangleMesh::buildBoundingBox()
{

    bounding_box.lb = vertices[0].cwiseMin(vertices[1]);
    bounding_box.ub = vertices[0].cwiseMax(vertices[1]);
    for (int i = 2; i < vertices.size(); i++)
    {
        bounding_box.lb = bounding_box.lb.cwiseMin(vertices[i]);
        bounding_box.ub = bounding_box.ub.cwiseMax(vertices[i]);
    }
	//ceil and floor
	for (int i = 0; i < 3; i++)
	{
		bounding_box.lb[i] = floor(bounding_box.lb[i]);
		bounding_box.ub[i] = ceil(bounding_box.ub[i]);
	}
}

void TriangleMesh::applyTransformation(const Eigen::Affine3f& t)
{
    for (int i = 0; i < vertices.size(); i++)
    {
        vertices[i] = t * vertices[i];
    }

    Eigen::Matrix3f t_inv_tr = t.linear().inverse().transpose();
    for (int i = 0; i < normals.size(); i++)
    {
        normals[i] = (t_inv_tr * normals[i]).normalized();
    }

    buildBoundingBox();
}

void TriangleMesh::buildUniformGrid() {
    /**
     * TODO: Build uniform grid here
     * Note: you may need to build your own data structures in the accel_struct.hpp and accel_struct.cpp
     */

	bounding_box.cellsize = 0.0125;
	float cellsize = bounding_box.cellsize;
	float xlength, ylength, zlength;
	float resol_x, resol_y, resol_z;

	xlength = bounding_box.getDist(0);
	ylength = bounding_box.getDist(1);
	zlength = bounding_box.getDist(2);

	resol_x = xlength / cellsize;
	resol_y = ylength / cellsize;
	resol_z = zlength / cellsize;

	//cout << bounding_box.lb << endl;
	//cout << bounding_box.ub << endl;
	//cout << endl;
	//
	//cout << resol_x << endl;
	//cout << resol_y << endl;
	//cout << resol_z << endl;

	UniformGrid grid;
	//construction of uniform grids
	vector<UniformGrid> z_grid;
	vector<vector<UniformGrid>> y_grid;

	for (float z = 0; z < resol_z; z++)
	{
		z_grid.push_back(grid);
	}
	for (float y = 0; y < resol_y; y++)
	{
		y_grid.push_back(z_grid);
	}
	for (float x = 0; x < resol_x; x++)
	{
		bounding_box.gridindex.push_back(y_grid);
	}

	//triangle intersection
	//for (int i = 0; i < 3; i += 3)
	for (int i = 0; i < vertices_index.size(); i += 3)
	{
		//get vertices of a triangle
		Eigen::Vector3f v1, v2, v3;

		v1 = vertices[vertices_index[i]];
		v2 = vertices[vertices_index[i+1]];
		v3 = vertices[vertices_index[i+2]];

		//get the aabb of this triangle
		AABB aabb(v1, v2, v3);
		//cout << aabb.lb << endl;
		//cout << aabb.ub << endl;

		//cellmin and cellmax for x,y,z
		int cellmin_x, cellmin_y, cellmin_z;
		int cellmax_x, cellmax_y, cellmax_z;
		cellmin_x = floor((aabb.lb[0] - bounding_box.lb[0]) / cellsize);
		cellmax_x = floor((aabb.ub[0] - bounding_box.lb[0]) / cellsize);

		cellmin_y = floor((aabb.lb[1] - bounding_box.lb[1]) / cellsize);
		cellmax_y = floor((aabb.ub[1] - bounding_box.lb[1]) / cellsize);

		cellmin_z = floor((aabb.lb[2] - bounding_box.lb[2]) / cellsize);
		cellmax_z = floor((aabb.ub[2] - bounding_box.lb[2]) / cellsize);

		for (int z = cellmin_z; z <= cellmax_z; z++)
		{
			for (int y = cellmin_y; y <= cellmax_y; y++)
			{
				for (int x = cellmin_x; x <= cellmax_x; x++)
				{
					//we only need to store the index of the first vertice
					bounding_box.gridindex[x][y][z].tri_index.push_back(i);
					//cout << bounding_box.gridindex[x][y][z].tri_index[0] << endl;
				}
			}
		}
	}

    has_grid = true;
}

void TriangleMesh::inform()
{
	cout << vertices.size() << endl;
	cout << vertices_index.size() << endl;
}
