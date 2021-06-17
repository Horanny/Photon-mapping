#include "Photon_Map.h"
const double INF = 1e8;
//#define swap(ph,a,b) {Photon * ph2 = ph[a]; ph[a] = ph[b]; ph[b] = ph2;}

Photon_map::Photon_map(int max_phot)
{
	//The constructor for the photon map
	//To create the photon map, it's necessary to specify the
	//maximum number of photons that will be stored
	stored_photons = 0;
	prev_scale = 1;
	max_photons = max_phot;

	//I'll use vector here
	//vector<Photon*> photons;
	photons = (Photon*)malloc(sizeof(Photon) * (max_photons + 1));

	if (photons == NULL)
	{
		std::cout << "Out of memory initializing photon map" << std::endl;
	}
	bbox_min = Eigen::Vector3f(INF, INF, INF);
	bbox_max = Eigen::Vector3f(-INF, -INF, -INF);
}

Photon_map::~Photon_map()
{
	free(photons);
}

void Photon_map::store(const Eigen::Vector3f power, const Eigen::Vector3f pos, const Eigen::Vector3f dir)
{
	if (stored_photons > max_photons)
		return;
	//store photons one by one
	stored_photons++;
	Photon* node = &photons[stored_photons];

	node->pos = pos;
	node->power = power;
	node->dir = dir;
	for (int i = 0; i < 3; i++)
	{
		if (node->pos[i] < bbox_min[i])
			bbox_min[i] = node->pos[i];
		if (node->pos[i] > bbox_max[i])
			bbox_max[i] = node->pos[i];
	}
}

void Photon_map::scale_photon_power(const float scale)
{
}

void Photon_map::balance(void)
{
	if (stored_photons > 1)
	{
		//allocate 2 temporary arrays for the balancing procedure
		Photon **pa1 = (Photon**)malloc(sizeof(Photon*)*(stored_photons + 1));
		Photon **pa2 = (Photon**)malloc(sizeof(Photon*)*(stored_photons + 1));
		
		for (int i = 0; i <= stored_photons; i++)
		{
			pa2[i] = &photons[i];
		}
		balance_segment(pa1, pa2, 1, 1, stored_photons);
		free(pa2);

		//reorganize balanced kd-tree(make a heap)
		int d, j = 1, foo = 1;
		Photon foo_photon = photons[j];

		for (int i = 1; i <= stored_photons; i++)
		{
			d = pa1[j] - photons;
			pa1[j] = NULL;
			if (d != foo)
				photons[j] = photons[d];
			else
			{
				photons[j] = foo_photon;

				if (i < stored_photons)
				{
					for (; foo <= stored_photons; foo++)
					{
						if (pa1[foo] != NULL)
							break;
						foo_photon = photons[foo];
						j = foo;
					}
					continue;
				}
				j = d;
			}
			free(pa1);
		}
		half_stored_photons = stored_photons / 2 - 1;
	}
}

void Photon_map::irradiance_estimate(Eigen::Vector3f irradiance, const Eigen::Vector3f pos, const Eigen::Vector3f normal, const float max_dist, const int nphotons) const
{
}

//finds the nearest photons in the photon map given the parameters in np
void Photon_map::locate_photons(NearestPhotons * np, const int index) const
{
	const Photon *p = &photons[index];
	float dist1;

	if (index < half_stored_photons)
	{
		dist1 = np->pos[p->plane] - p->pos[p->plane];

		if (dist1 > 0.0)
		{
			//if dist1 is positive search right plane
			locate_photons(np, 2 * index + 1);
			if (dist1 * dist1 < np->dist2[0])
				locate_photons(np, 2 * index);
		}
		else
		{
			//dist1 is negative search left first
			locate_photons(np, 2 * index);
			if (dist1*dist1 < np->dist2[0])
				locate_photons(np, 2 * index + 1);
		}
	}

	//compute sauared distance between current photon and np->pos
	dist1 = p->pos[0] - np->pos[0];
	float dist2 = dist1 * dist1;
	dist1 = p->pos[1] - np->pos[1];
	dist2 += dist1 * dist1;
	dist1 = p->pos[2] - np->pos[2];
	dist2 += dist1 * dist1;
	//dist2 = delta^2 = squared distance from photon p to x

	if (dist2 < np->dist2[0])
	{
		//we found a photon, insert it in the candidate list
		if (np->found < np->max)
		{
			//heap is not full
			np->found++;
			np->dist2[np->found] = dist2;
			np->index[np->found] = p;
		}
		else
		{
			int j, parent;

			if (np->got_heap == 0)
			{
				//build heap
				float dst2;
				const Photon *phot;
				// /2
				int half_found = np->found >> 1;
				for (int k = half_found; k >= 1; k--)
				{
					parent = k;
					phot = np->index[k];
					dst2 = np->dist2[k];
					while (parent <= half_found)
					{
						j = parent + parent;
						if (j < np->found && np->dist2[j] < np->dist2[j + 1])
							j++;
						if (dst2 >= np->dist2[j])
							break;
						np->dist2[parent] = np->dist2[j];
						np->index[parent] = np->index[j];
						parent = j;
					}
					np->got_heap = 1;
				}
			}

			//insert new photon into max heap
			//delete largest element, insert new, and reorder the heap
			parent = 1;
			j = 2;
			while (j <= np->found)
			{
				if (j < np->found && np->dist2[j] < np->dist2[j + 1])
					j++;
				if (dist2 > np->dist2[j])
					break;
				np->dist2[parent] = np->dist2[j];
				np->index[parent] = np->index[j];
				parent = j;
				j += j;
			}
			np->index[parent] = p;
			np->dist2[parent] = dist2;
			np->dist2[0] = np->dist2[1];
		}
		
	}
}

void Photon_map::balance_segment(Photon ** pbal, Photon ** porg, const int index, const int start, const int end)
{
	/////////////////////////////////
	//////////////WHY/////////////
	//compute new median
	int median = 1;
	while((4 * median) <= (end - start + 1))
		median += median;

	if ((3 * median) <= (end - start + 1))
	{
		median += median;
		median += start - 1;
	}
	else
		median = end - median + 1;

	//find axis to split along
	int axis = 2;
	if ((bbox_max[0] - bbox_min[0]) > (bbox_max[1] - bbox_min[1]) &&
		(bbox_max[0] - bbox_min[0]) > (bbox_max[2] - bbox_min[2]))
		axis = 0;
	else if ((bbox_max[1] - bbox_min[1]) > (bbox_max[2] - bbox_min[2]))
		axis = 1;

	//partition photon block around the median
	median_split(porg, start, end, median, axis);
	pbal[index] = porg[median];
	pbal[index]->plane = axis;

	//recursively balance the left and right block
	if (median > start)
	{
		//balance legt segment
		if (start < median - 1)
		{
			const float tmp = bbox_max[axis];
			bbox_max[axis] = pbal[index]->pos[axis];
			balance_segment(pbal, porg, 2 * index, start, median - 1);
			bbox_max[axis] = tmp;
		}
		else
		{
			pbal[2 * index] = porg[start];
		}
	}
	if (median < end)
	{
		//balance right segment
		if (median + 1< end)
		{
			const float tmp = bbox_min[axis];
			bbox_min[axis] = pbal[index]->pos[axis];
			balance_segment(pbal, porg, 2 * index + 1, median+1,end);
			bbox_max[axis] = tmp;
		}
		else
		{
			pbal[2 * index+1] = porg[end];
		}
	}
}

////////////WHY/////////////
void Photon_map::median_split(Photon ** p, const int start, const int end, const int median, const int axis)
{
	//this function splits the photon array into 2 separate
	//pieces around the median, with all photons below the
	//median in the lower half and all photons above the
	//median in the upper half.
	//The comparison criteria is the axis(indicated by the axis parameter)
	int left = start;
	int right = end;

	while (right > left)
	{
		const float v = p[right]->pos[axis];
		int i = left - 1;
		int j = right;
		for (;;)
		{
			while (p[++i]->pos[axis] < v)
				;
			while (p[--j]->pos[axis] > v && j > left)
				;
			if (i >= j)
				break;
			std::swap(p[i], p[j]);
		}
		std::swap(p[i], p[right]);
		if (i >= median)
			right = i - 1;
		if (i <= median)
			left = i + 1;
	}
}
