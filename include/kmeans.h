#ifndef KMEANS_H
#define KMEANS_H

#include <vector>
#include <eigen3/Eigen/Core>

namespace LIDAR{
	
class Kmeans{
public:
	Kmeans(){}
	
	static void clustering(const std::vector< Eigen::Vector3d >& point_cloud,
						int k, int max_iters, double epsilon,
						std::vector< int >& classes // flag for each point.
	);
	
}; // class Kmeans
	
	
} //namespace LIDAR

#endif
