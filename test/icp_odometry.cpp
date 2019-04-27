#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv4/opencv2/opencv.hpp>
#include <icp.h>
#include <eigen3/Eigen/Dense>

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
    cv::Mat imD_last, imRGB_last;

    // Two points clouds.
    std::vector<Eigen::Vector3d> pts_model;
    std::vector<Eigen::Vector3d> pts_cloud;
    pts_model.reserve ( rows*cols );
    pts_cloud.reserve ( rows*cols );

    // Global pose of the camera.
    Eigen::Matrix3d Rwc = Eigen::Matrix3d::Identity();
    Eigen::Vector3d twc ( 0.0, 0.0, 0.0 );

    // Traj file.
    ofstream f;
    f.open ( "traj.txt" );
    f << fixed;

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

        // using ICP.
        if ( !imD.empty() && !imD_last.empty() ) {
            pts_model.clear();
            pts_cloud.clear();

            // Get two Clouds
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

            for ( int v = 0; v < imD_last.rows; v++ ) {
                for ( int u = 0; u < imD_last.cols; u ++ ) {
                    unsigned int d = imD_last.ptr<unsigned short> ( v ) [u];
                    if ( d == 0 ) {
                        continue;
                    }

                    double z = d * depth_factor;
                    double x = ( u-cx ) * z / fx;
                    double y = ( v-cy ) * z / fy;

                    pts_model.push_back ( Eigen::Vector3d ( x,y,z ) );
                } // for all u
            } // for all v


            // find transformation by ICP
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            Eigen::Vector3d t ( 0.0,0.0,0.0 );
            ICP::findTransformation ( pts_model, pts_cloud, 100, 1.0e-10, 1.0e-10, R, t );

            // conver local R t to global pose of the camera Rwc, twc
            Rwc = Rwc * R;
            twc = Rwc * t + twc;

            // convert R to Quaternion.
            Eigen::Quaterniond q ( R );

            // save poses to file.
            f << setprecision ( 6 ) << tframe << " " <<  setprecision ( 9 ) << twc[0] << " " << twc[1] << " " << twc[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

            // print poses.
            std::cout << std::fixed << setprecision ( 6 ) << tframe << " " <<  setprecision ( 9 ) << twc[0] << " " << twc[1] << " " << twc[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

            // show two images.
            cv::imshow ( "Cur RGB", imRGB );
            cv::imshow ( "Last RGB", imRGB_last );
            cv::waitKey ( 1 );
        } // using ICP.

        imD_last = imD.clone();
        imRGB_last = imRGB.clone();
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
