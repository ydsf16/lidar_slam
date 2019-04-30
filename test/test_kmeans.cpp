#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv4/opencv2/opencv.hpp>
#include <icp.h>
#include <kmeans.h>
#include <eigen3/Eigen/Dense>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace LIDAR;

void LoadImages ( const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
				  vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps );

int main ( int argc, char **argv )
{
	if ( argc != 3 ) {
		std::cerr << endl << "Usage: ./icp_odometry path_to_sequence path_to_association" << endl;
		return -1;
	}
	
	// camera params
	const double depth_factor = 1.0 / 5000.0;
	const double fx = 517.306408;
	const double fy = 516.469215;
	const double cx = 318.643040;
	const double cy =  255.313989;
	const int rows = 480;
	const int cols = 640;
	const std::vector<uint8_t> colors = {213,0,0,197,17,98,170,0,255,98,0,234,48,79,254,41,98,255,0,145,234,0,184,212,0,191,165,0,200,83,100,221,23,174,234,0,255,214,0,255,171,0,255,109,0,221,44,0,62,39,35,33,33,33,38,50,56,144,164,174,224,224,224,161,136,127,255,112,67,255,152,0,255,193,7,255,235,59,192,202,51,139,195,74,67,160,71,0,150,136,0,172,193,3,169,244,100,181,246,63,81,181,103,58,183,171,71,188,236,64,122,239,83,80, 213,0,0,197,17,98,170,0,255,98,0,234,48,79,254,41,98,255,0,145,234,0,184,212,0,191,165,0,200,83,100,221,23,174,234,0,255,214,0,255,171,0,255,109,0,221,44,0,62,39,35,33,33,33,38,50,56,144,164,174,224,224,224,161,136,127,255,112,67,255,152,0,255,193,7,255,235,59,192,202,51,139,195,74,67,160,71,0,150,136,0,172,193,3,169,244,100,181,246,63,81,181,103,58,183,171,71,188,236,64,122,239,83,80};
	
	
	vector<string> vstrImageFilenamesRGB;
	vector<string> vstrImageFilenamesD;
	vector<double> vTimestamps;
	string strAssociationFilename = string ( argv[2] );
	LoadImages ( strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps );
	// Check consistency in the number of images and depthmaps
	int nImages = vstrImageFilenamesRGB.size();
	if ( vstrImageFilenamesRGB.empty() ) {
		cerr << endl << "No images found in provided path." << endl;
		return 1;
	} else if ( vstrImageFilenamesD.size() !=vstrImageFilenamesRGB.size() ) {
		cerr << endl << "Different number of images for rgb and depth." << endl;
		return 1;
	}
	
	
	// Main loop
	cv::Mat imRGB, imD;

	// Two points clouds.
	std::vector<Eigen::Vector3d> pts_cloud;
	pts_cloud.reserve ( rows*cols );
	
	// pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	pcl::visualization::CloudViewer viewer ("Clusters");
	
	for ( int ni=0; ni<nImages; ni++ ) {
		// Read image and depthmap from file
		imRGB = cv::imread ( string ( argv[1] ) +"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED );
		imD = cv::imread ( string ( argv[1] ) +"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED );
		double tframe = vTimestamps[ni];
		
		if ( imRGB.empty() ) {
			cerr << endl << "Failed to load image at: "
			<< string ( argv[1] ) << "/" << vstrImageFilenamesRGB[ni] << endl;
			return 1;
		}
		
		// show image.
		cv::imshow ( "Cur RGB", imRGB );
		cv::waitKey ( 1 );
		
		// Get point cloud. 
		pts_cloud.clear();
		for ( int v = 0; v < imD.rows; v++ ) {
			for ( int u = 0; u < imD.cols; u ++ ) {
				unsigned int d = imD.ptr<unsigned short> ( v ) [u];
				if ( d == 0 ) {
					continue;
				}
				
				double z = d * depth_factor;
				double x = ( u-cx ) * z / fx;
				double y = ( v-cy ) * z / fy;
				
				pts_cloud.push_back ( Eigen::Vector3d ( x,y,z ) );
			} // for all u
		} // for all v
		

		// segmente by kmeans
		std::vector<int> classes;
		Kmeans::clustering(pts_cloud, 10, 100, 1e-5, classes);
		
		// Show classes.
 		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		
		for(size_t i = 0; i < pts_cloud.size(); i ++)
		{
			const Eigen::Vector3d& pte = pts_cloud.at(i);
			const int& class_id = classes.at(i);
			pcl::PointXYZRGB ptc;
			ptc.x = pte[0];
			ptc.y = pte[1];
			ptc.z = pte[2];
			
			ptc.r = colors.at(class_id*3);
			ptc.g = colors.at(class_id*3+1);
			ptc.b = colors.at(class_id*3+2);
			cloud->points.push_back(ptc);
		}
		viewer.showCloud (cloud);
		
	} // for all images.
	
	return 0;
}




void LoadImages ( const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
				  vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps )
{
	ifstream fAssociation;
	fAssociation.open ( strAssociationFilename.c_str() );
	while ( !fAssociation.eof() ) {
		string s;
		getline ( fAssociation,s );
		if ( !s.empty() ) {
			stringstream ss;
			ss << s;
			double t;
			string sRGB, sD;
			ss >> t;
			vTimestamps.push_back ( t );
			ss >> sRGB;
			vstrImageFilenamesRGB.push_back ( sRGB );
			ss >> t;
			ss >> sD;
			vstrImageFilenamesD.push_back ( sD );
		}
	}
}
