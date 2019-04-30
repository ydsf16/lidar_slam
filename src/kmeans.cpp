// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <kmeans.h>
#include <opencv4/opencv2/opencv.hpp>

namespace LIDAR
{

void Kmeans::clustering ( const std::vector< Eigen::Vector3d >& point_cloud, int k, int max_iters, double epsilon, std::vector< int >& classes )
{
    if ( point_cloud.size() < k ) {
        return;
    }
    //classes.reserve ( point_cloud.size() );
    //classes.shrink_to_fit();
    classes = std::vector<int> ( point_cloud.size(), 0 );

    // step 1. rand select k points. TODO
    std::vector<Eigen::Vector3d> means; // means for k
    means.reserve ( k );

    cv::RNG rng ( cv::getTickCount() );
    std::set<int> selected_ids;
    while ( selected_ids.size() < k ) {
        int id = rng.uniform ( 0, point_cloud.size() );
        if ( ! selected_ids.count ( id ) ) {
            selected_ids.insert ( id );
        }
    } // select ids.
    std::set<int>::const_iterator iter;
    for ( iter = selected_ids.begin(); iter != selected_ids.end(); ++iter ) {
        means.push_back ( point_cloud.at ( *iter ) );
    }

    // start loop
    for ( size_t nit = 0; nit < max_iters; ++nit ) {
        // step 2. assign points to classes.
        // for each point
        for ( size_t i = 0; i < point_cloud.size(); i ++ ) {
            // pt.
            const Eigen::Vector3d& pt = point_cloud.at ( i );

            // init params
            int min_class_id = -1;
            double min_dist = std::numeric_limits<double>::max();

            for ( size_t j = 0; j < k ; j ++ ) {
                const Eigen::Vector3d& mean = means.at ( j );
				double dist = ( pt-mean ).norm(); // Euclidean distance
                if ( dist < min_dist ) {
                    min_dist = dist;
                    min_class_id = j;
                }
            }

            classes.at ( i ) = min_class_id; // assign id.
        }// for each point assign to the nearest class.

        // step 3. compute new center for each class.
        std::vector<Eigen::Vector3d> new_means ( k, Eigen::Vector3d ( 0.0,0.0,0.0 ) ); // sum of each
        std::vector<int> npts_class ( k, 0 ); // number of points of each class
        for ( size_t i = 0; i < point_cloud.size(); i ++ ) {
            // pt
            const Eigen::Vector3d& pt = point_cloud.at ( i );
            const int& class_id = classes.at ( i );

            // sum
            new_means.at ( class_id ) += pt;
            npts_class.at ( class_id ) ++;
        }

        for ( size_t i = 0; i < k; i ++ ) {
            new_means.at ( i ) /= ( double ) ( npts_class.at ( i ) );
        }

        // Step 4. check if it has converged
        double delta_mean = 0.0;
        for ( size_t i = 0; i < k; i ++ ) {
            delta_mean += ( new_means.at ( i ) - means.at ( i ) ).norm();
        }

        if ( delta_mean < epsilon ) {
            break;
        }

        // copy means.
        means = new_means;
    } // for max_iters.
    
   

} //  clustering


} // namespace LIDAR
