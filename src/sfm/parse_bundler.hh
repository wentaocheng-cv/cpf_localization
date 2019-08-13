/*===========================================================================*\
 *                                                                           *
 *                            ACG Localizer                                  *
 *      Copyright (C) 2011-2012 by Computer Graphics Group, RWTH Aachen      *
 *                           www.rwth-graphics.de                            *
 *                                                                           *
 *---------------------------------------------------------------------------* 
 *  This file is part of ACG Localizer                                       *
 *                                                                           *
 *  ACG Localizer is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  ACG Localizer is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with ACG Localizer.  If not, see <http://www.gnu.org/licenses/>.   *
 *                                                                           *
\*===========================================================================*/ 

#ifndef PARSE_BUNDLER_HH
#define PARSE_BUNDLER_HH

/**
 * Parses a bundle.out file created by Bundler into a set of 
 * 3D points and the corresponding descriptors.
 *
 * author: Torsten Sattler (tsattler@cs.rwth-aachen.de)
 * date  : 11-08-2012
**/

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdint.h>
#include <cstdlib>
#include <string>
#include "../exif_reader/exif_reader.hh"
#include "../features/SIFT_keypoint.hh"
#include "../features/SIFT_loader.hh"
#include "bundler_camera.hh"
#include <math.h>




////////////////////////////
// Simple definition of a 3D point
// consisting of x,y,z coordinates
// and a point color.
////////////////////////////
class point3D
{
  public:
    float x,y,z;
    
    unsigned char r,g,b;
    
    point3D()
    {
      x = y = z = 0.0;
      r = g = b = 0;
    }
    
    point3D( float x_, float y_, float z_ )
    {
      x = x_;
      y = y_;
      z = z_;
      r = g = b = 0;
    }
    
    point3D( float x_, float y_, float z_, unsigned char r_, unsigned char g_, unsigned char b_ )
    {
      x = x_;
      y = y_;
      z = z_;
      r = r_;
      g = g_;
      b = b_;
    }
    
    point3D( const point3D &other )
    {
      x = other.x;
      y = other.y;
      z = other.z;
      r = other.r;
      g = other.g;
      b = other.b;
    }
    
    
    void operator=( const point3D &other )
    {
      x = other.x;
      y = other.y;
      z = other.z;
      r = other.r;
      g = other.g;
      b = other.b;
    }
};

////////////////////////////
// A view of a 3D point as defined by Bundler. It
// contains the index of the camera the point was seen it,
// the keypoint index in the .key file of that camera corresponding
// to the projection of the 3D point, the x- and y-positions of that keypoint 
// in a reference frame where the center of the coordinate system is the center
// of the cameras image, as well as the scale and orientation of that keypoint.
// (see http://phototour.cs.washington.edu/bundler/bundler-v0.4-manual.html)
////////////////////////////
struct view
{
  uint32_t camera; // index of the camera this feature was detected in
  uint32_t key; // index of this feature in the corresponding .key file containing the cameras SIFT features
  float x,y;
  float scale; // scale at which the feature was detected as taken from the original image
  float orientation; // orientation of that features as detected in the image
  //wentao cheng
  int width;
  int height;

    
  view()
  {
    camera = key = 0;
    x = y = 0.0;
    scale = 0.0f;
    orientation = 0.0f;
  }
};


////////////////////////////
// Information about a reconstructed 3D point. The structure contains information about the 3D position
// of the point, its color, and a list of views in which the point can be seen and the corresponding SIFT descriptors.
// The descriptor for view view_list[i] is stored in descriptors[128*i]...descriptors[128*i+127]
////////////////////////////
class feature_3D_info
{
  public:
    point3D point;
    std::vector< view > view_list; 
    std::vector< int > local_neighbors; 
    int max_local_size;

    std::vector< unsigned char > descriptors;

    //cheng wentao
    float gain;
    bool s_flag;
    uint32_t confidence;
    std::vector< unsigned char > mean;
    float f_localness;
    float f_prior_dist;
    //the view angle from each db camera
    std::vector< float > f_angle;
    std::vector< float > matched_query;

    //cheng wentao
    
    // constructor
    feature_3D_info( )
    {
      view_list.clear();
      descriptors.clear();
    }
    
    // copy constructor
    feature_3D_info( const feature_3D_info &other )
    {
      view_list.clear();
      descriptors.clear();
      view_list.resize( other.view_list.size() );
      descriptors.resize( other.descriptors.size() );
      
      for( uint32_t i=0; i< (uint32_t) other.view_list.size(); ++i )
      {
        view_list[i].camera = other.view_list[i].camera;
        view_list[i].key = other.view_list[i].key;
        view_list[i].x = other.view_list[i].x;
        view_list[i].y = other.view_list[i].y;
        view_list[i].scale = other.view_list[i].scale;
        view_list[i].orientation = other.view_list[i].orientation;
      }
      
      for( uint32_t i=0; i<(uint32_t)other.descriptors.size(); ++i )
        descriptors[i] = other.descriptors[i];
      
      point = other.point;
    }
    
    // destructor
    ~feature_3D_info( )
    {
      view_list.clear();
      descriptors.clear();
    }
    
    // reset all data
    void clear_data()
    {
      view_list.clear();
      descriptors.clear();
    }
};

////////////////////////////
// Class to parse the output files generated by Bundler.
// The class is able to further load the .key files of all cameras in
// the reconstruction given that the .key files are not zipped (Bundler
// generally uses gzip to compress the .key files). It can also
// load the file format generated by Bundle2Info.
////////////////////////////
class parse_bundler
{
  public:
    
    // standard constructor
    parse_bundler();
    
    // standard destructor
    ~parse_bundler( );
    
    // get the number of 3D points in the reconstruction
    uint32_t get_number_of_points( );

    // get the number of cameras in the reconstruction
    uint32_t get_number_of_cameras( );
    
    // get the actual 3D points from the loaded reconstruction
    void get_points( std::vector< point3D > &points );
    
    // get the feature information extracted from the reconstruction
    std::vector< feature_3D_info >& get_feature_infos( );
    
    // get the cameras included in the reconstruction
    std::vector< bundler_camera >& get_cameras( );
    
    
    // Parses the output text file generated by bundler.
    // Input parameters: The filename of the output file (usually bundle.out) and
    // the filename of the list of images (usually list.txt).
    bool parse_data( const char* bundle_out_filename_, const char* image_list_filename );

    bool parse_camera_data( const char* bundle_out_filename_);
    bool parse_data_nokey( const char* bundle_out_filename_);
    
    // Load the information from a binary file constructed with Bundle2Info.
    // The format parameter specifies whether the binary file contains 
    // no camera information (format=0, as generated by Bundle2Info) or does 
    // contain camera information (format=1, for example the aachen.info file 
    // released with the Aachen dataset from the paper
    //
    // Torsten Sattler, Tobias Weyand, Bastian Leibe, Leif Kobbelt. 
    // Image Retrieval for Image-Based Localization Revisited.
    // BMVC 2012.
    bool load_from_binary( const char* filename, const int format );

    bool load_from_binary_nokey( const char* filename, const int format );

    bool parse_data_from_binary( const char *bundle_out_filename_,
        std::vector< feature_3D_info > &feature_infos, const char *id_file );
    
    // clear the loaded data
    void clear();
    
  private:
    
    std::vector< bundler_camera > mCameras;
    
    std::vector< feature_3D_info > mFeatureInfos;
    
    uint32_t mNbPoints, mNbCameras;
    
};



//implementation of the kmeans method
class Kmeans_Point
{
  public:
    float x;
    float y;
    float z;
    int id;
    int id_in_match;
    int belong_center;
};


class ClusterCenter
{
public:
    //members
  //change all point list into vetorf
    std::vector <Kmeans_Point> in_points;
    //center
    float center_x;
    float center_y;
    float center_z;
    float avg_distance;
    std::vector <int> vis_list;

    void compute_centroid()
    {
        float x = 0; float y = 0; float z = 0;
        int points_number = (int) in_points.size();
        for (int i = 0; i < points_number; i++)
        {
            x += in_points[i].x;
            y += in_points[i].y;
            z += in_points[i].z;
        }
        center_x = x / float(points_number);
        center_y = y / float(points_number);
        center_z = z / float(points_number);
    }

    void compute_average_distance()
    {
        int points_number = (int) in_points.size();
        avg_distance = 0;
        for (int i = 0; i < points_number; i++)
        {
            float pow_dist = (in_points[i].x - center_x) * (in_points[i].x - center_x)
                             + (in_points[i].y - center_y) * (in_points[i].y - center_y)
                             + (in_points[i].z - center_z) * (in_points[i].z - center_z) ;
            avg_distance += sqrt(pow_dist);
        }
        avg_distance /= float(points_number);
    }

};



class Kmeans
{
public:
    //number of cluster centers
    int mClusters;
    //number of iterations;
    int mIterations;
    //error tolerance
    float mError;
    //center
    std::vector <ClusterCenter> cCenters;


    //initialize the clusters, from the selected points.
    //randomly????
    void Init(std::vector<Kmeans_Point> &process_points, int numberCluster, int max_iteration)
    {
        //std::cout << ".." << std::endl;
        mClusters = numberCluster;
        mIterations = max_iteration;
        //reserve the center list
        cCenters.resize(numberCluster);
        for(int i = 0; i<numberCluster;i++)
        {
          cCenters[i].in_points.clear();
        }
        //get the size of current selected points.
        int list_len = (int) process_points.size();
        //std::cout << "read " << list_len << " for kmeans" << std::endl;



        //randomly pick cluster center.
        //std::cout << "randomly pick the cluster center " << std::endl;
        std::vector< bool > flag_pick; 
        flag_pick.reserve(list_len);
        for (int i = 0; i < list_len; i++)
        {
            flag_pick[i] = false;
        }
        int curr_cluster = 0;

        bool loop_init = true;
        srand (time(NULL));
        while (loop_init)
        {
            int poss_pos = rand() % list_len;
            //std::cout << poss_pos << std::endl;
            if (flag_pick[poss_pos] == false)
            {
                cCenters[curr_cluster].in_points.push_back(process_points[poss_pos]);
                cCenters[curr_cluster].center_x = process_points[poss_pos].x;
                cCenters[curr_cluster].center_y = process_points[poss_pos].y;
                cCenters[curr_cluster].center_z = process_points[poss_pos].z;
                //std::cout << cCenters[curr_cluster].in_points.size() << std::endl;
                curr_cluster++;
                flag_pick[poss_pos] = true;
            }
            if (curr_cluster >= numberCluster)
            {
                loop_init = false;
            }
        }
        //std::cout << " done " << curr_cluster << std::endl;
    }

    //cluster process
    void Cluster(std::vector<Kmeans_Point> &process_points, int numberCluster, int max_iteration, float centroid_threshold)
    {
        //initialize the list
        Init(process_points, numberCluster, max_iteration);
        //get the size of current selected points.
       // std::cout << "* " << std::endl;
        int list_len = (int) process_points.size();
        std::vector<ClusterCenter> temp_cluster;
       // std::cout << "** " << std::endl;
        temp_cluster.resize(numberCluster);

        //std::cout << "start kmeans " << std::endl;
        bool loop = true;
        int count = 0;
        //std::cout << "**** " << std::endl;
        while (loop)
        {
            int unchanged = 0;
            //load the old cluster centers

            for (int i = 0; i < numberCluster; i++)
            {
               // std::cout << "***** " << i << list_len << std::endl;
                temp_cluster[i].in_points.clear();
            }
            //std::cout << ". " << std::endl;
            //classification
            for (int i = 0; i < list_len; i++)
            {
                float min_dist = INFINITY;
                int assign_to_center = 0;
                for (int j = 0; j < numberCluster; j++)
                {
                    float dist = (process_points[i].x - cCenters[j].center_x) * (process_points[i].x - cCenters[j].center_x)
                                 + (process_points[i].y - cCenters[j].center_y) * (process_points[i].y - cCenters[j].center_y)
                                 + (process_points[i].z - cCenters[j].center_z) * (process_points[i].z - cCenters[j].center_z);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        assign_to_center = j;
                    }
                }
                //assign this point to the new center.
                process_points[i].belong_center = assign_to_center;
                temp_cluster[assign_to_center].in_points.push_back(process_points[i]);
               
            }
            //std::cout << ".. " << std::endl;
            //update the centroid of the new clusters
            for (int i = 0; i < numberCluster; i++)
            {
                temp_cluster[i].compute_centroid();
                //check whether this centroid has changed above the threshold
                float centroid_change = (temp_cluster[i].center_x - cCenters[i].center_x) * (temp_cluster[i].center_x - cCenters[i].center_x)
                                        + (temp_cluster[i].center_y - cCenters[i].center_y) * (temp_cluster[i].center_y - cCenters[i].center_y)
                                        + (temp_cluster[i].center_z - cCenters[i].center_z) * (temp_cluster[i].center_z - cCenters[i].center_z);
                if (sqrt(centroid_change) <= centroid_threshold)
                    unchanged++;
            }

            //std::cout << "... " << std::endl;
            //swap the current cluster center with the new center
            for (int i = 0; i < numberCluster; i++)
            {
                //cCenters.clear();
                int new_size = (int) temp_cluster[i].in_points.size();
                cCenters[i].in_points.clear();
                //cCenters[i].in_points.resize(new_size);
                for (int j = 0; j < new_size; j++)
                {
                    cCenters[i].in_points.push_back(temp_cluster[i].in_points[j]);
                }
                cCenters[i].center_x = temp_cluster[i].center_x;
                cCenters[i].center_y = temp_cluster[i].center_y;
                cCenters[i].center_z = temp_cluster[i].center_z;
            }
            //std::cout << ".... " << std::endl;
           // std::cout << "finish the " << count << " round, unchanged " << unchanged << std::endl;
            count++;
            if (count == mIterations && centroid_threshold == 0.0f)
                loop = false;
            else if (unchanged >= 0.95 * mClusters && centroid_threshold != 0.0f)
                loop = false;
        }
    }
};

#endif 
