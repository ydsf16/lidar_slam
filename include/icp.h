
#ifndef ICP_H
#define ICP_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

namespace LIDAR
{
class ICP{
public:
	ICP();
	
	static void findTransformation(const std::vector<Eigen::Vector3d>& pts_model,
		const std::vector<Eigen::Vector3d>& pts_cloud,
		double n_iters, double epsilon, double min_err,
		Eigen::Matrix3d& R, Eigen::Vector3d& t
	);
	
}; //class ICP
	
	
} // namespace LIDAR

#endif
