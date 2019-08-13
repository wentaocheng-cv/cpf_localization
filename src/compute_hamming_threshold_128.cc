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


#define __STDC_LIMIT_MACROS

// C++ includes
#include <bitset>
#include <vector>
#include <set>
#include <map>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <stdint.h>
#include <string>
#include <algorithm>
#include <climits>
#include <float.h>
#include <cmath>
#include <sstream>
#include <stdio.h>

// includes for classes dealing with SIFT-features
#include "features/SIFT_loader.hh"
#include "features/visual_words_handler.hh"
#include "sfm/parse_bundler.hh"
// stopwatch
#include "timer.hh"


// exif reader to get the width and height out
// of the exif tags of an image
#include "exif_reader/exif_reader.hh"


const uint64_t sift_dim = 128;

////
// Classes to handle the two nearest neighbors (nn) of a descriptor.
// There are three classes:
// 1. Normal 2 nearest neighbors for integer distances
// 2. 2 nearest neighbors for integer distances, making sure
//    that the second nearest neighbor does not belong to the same 3D point
// 3. Normal 2 nearest neighbors for floating point distances
//
// We store the distances to the 2 nearest neighbors as well as the ids of the
// corresponding 3D points and update the 2 nearest neighbors if needed.
// The stored distances are squared Euclidean distances.
////

// for unsigned char descriptors
class nearest_neighbors
{
public:

  // constructors
  nearest_neighbors()
  {
    nn_idx1 = nn_idx2 = UINT32_MAX;
    dist1 = dist2 = -1;
  }

  nearest_neighbors( uint32_t nn1, uint32_t nn2, int d1, int d2 ) : nn_idx1( nn1 ), nn_idx2( nn2 ), dist1( d1 ), dist2( d2 )
  {}

  nearest_neighbors( uint32_t nn1, int d1 ) : nn_idx1( nn1 ), nn_idx2( UINT32_MAX ), dist1( d1 ), dist2( -1 )
  {}

  nearest_neighbors( const nearest_neighbors &other )
  {
    if ( &other != this )
    {
      nn_idx1 = other.nn_idx1;
      nn_idx2 = other.nn_idx2;
      dist1 = other.dist1;
      dist2 = other.dist2;
    }
  }

  // update the 2 nn with a new distance to a 3D points
  void update( uint32_t point, int dist )
  {
    if ( dist1 < 0 )
    {
      nn_idx1 = point;
      dist1 = dist;
    }
    else
    {
      if ( dist < dist1 )
      {
        nn_idx2 = nn_idx1;
        dist2 = dist1;
        nn_idx1 = point;
        dist1 = dist;
      }
      else if ( dist < dist2 || dist2 < 0 )
      {
        nn_idx2 = point;
        dist2 = dist;
      }
    }
  }

  float get_ratio()
  {
    return float(dist1) / float(dist2);
  }

  uint32_t nn_idx1, nn_idx2;
  int dist1, dist2;
};

// for the case that multiple descriptors of the same 3D point are mapped to the same visual word
class nearest_neighbors_multiple
{
public:
  nearest_neighbors_multiple()
  {
    nn_idx1 = nn_idx2 = UINT32_MAX;
    dist1 = dist2 = -1;
  }

  nearest_neighbors_multiple( uint32_t nn1, uint32_t nn2, int d1, int d2 ) : nn_idx1( nn1 ), nn_idx2( nn2 ), dist1( d1 ), dist2( d2 )
  {}

  nearest_neighbors_multiple( uint32_t nn1, int d1 ) : nn_idx1( nn1 ), nn_idx2( UINT32_MAX ), dist1( d1 ), dist2( -1 )
  {}

  nearest_neighbors_multiple( const nearest_neighbors_multiple &other )
  {
    if ( &other != this )
    {
      nn_idx1 = other.nn_idx1;
      nn_idx2 = other.nn_idx2;
      dist1 = other.dist1;
      dist2 = other.dist2;
    }
  }

  void update( uint32_t point, int dist )
  {
    if ( dist1 < 0 )
    {
      nn_idx1 = point;
      dist1 = dist;
    }
    else
    {
      if ( dist < dist1 )
      {
        if ( nn_idx1 != point )
        {
          nn_idx2 = nn_idx1;
          dist2 = dist1;
        }
        nn_idx1 = point;
        dist1 = dist;
      }
      else if ( (dist < dist2 || dist2 < 0) && (point != nn_idx1) )
      {
        nn_idx2 = point;
        dist2 = dist;
      }
    }
  }

  float get_ratio()
  {
    return float(dist1) / float(dist2);
  }

  uint32_t nn_idx1, nn_idx2;
  int dist1, dist2;
};

// for float descriptors
class nearest_neighbors_float
{
public:
  nearest_neighbors_float()
  {
    nn_idx1 = nn_idx2 = UINT32_MAX;
    dist1 = dist2 = -1.0;
  }

  nearest_neighbors_float( uint32_t nn1, uint32_t nn2, float d1, float d2 ) : nn_idx1( nn1 ), nn_idx2( nn2 ), dist1( d1 ), dist2( d2 )
  {}

  nearest_neighbors_float( uint32_t nn1, float d1 ) : nn_idx1( nn1 ), nn_idx2( UINT32_MAX ), dist1( d1 ), dist2( -1 )
  {}

  nearest_neighbors_float( const nearest_neighbors_float &other )
  {
    if ( &other != this )
    {
      nn_idx1 = other.nn_idx1;
      nn_idx2 = other.nn_idx2;
      dist1 = other.dist1;
      dist2 = other.dist2;
    }
  }

  void update( uint32_t point, float dist )
  {
    if ( dist1 < 0 )
    {
      nn_idx1 = point;
      dist1 = dist;
    }
    else
    {
      if ( dist < dist1 )
      {
        nn_idx2 = nn_idx1;
        dist2 = dist1;
        nn_idx1 = point;
        dist1 = dist;
      }
      else if ( dist < dist2 || dist2 < 0 )
      {
        nn_idx2 = point;
        dist2 = dist;
      }
    }
  }

  float get_ratio()
  {
    return dist1 / dist2;
  }

  uint32_t nn_idx1, nn_idx2;
  float dist1, dist2;
};

////
// functions to compute the squared distances between two SIFT-vectors
// there are different ways how the SIFT-vectors are stored, for each there is
// one function
////

// First descriptor is stored in an array, while the second descriptor is stored in a vector (concatenation of vector entries)
// The second descriptor begins at position index*128
inline int compute_squared_SIFT_dist( const unsigned char * const v1, std::vector< unsigned char > &v2, uint32_t index )
{
  uint64_t index_( index );
  index_ *= sift_dim;
  int dist = 0;
  int x = 0;
  for ( uint64_t i = 0; i < sift_dim; ++i )
  {
    x = int( v1[i] ) - int( v2[index_ + i] );
    dist += x * x;
  }
  return dist;
}

// same in case that one descriptors consists of floating point values
inline float compute_squared_SIFT_dist_float( const unsigned char * const v1, std::vector< float > &v2, uint32_t index )
{
  size_t index_( index );
  index_ *= sift_dim;
  float dist = 0;
  float x = 0;
  for ( int i = 0; i < sift_dim; ++i )
  {
    x = float( v1[i] ) - v2[index_ + i];
    dist += x * x;
  }
  return dist;
}


// function to sort (2D feature, visual word) point pairs for the prioritized search.
inline bool cmp_priorities( const std::pair< uint32_t, double >& a, const std::pair< uint32_t, double >& b )
{
  return ( a.second < b.second );
}

////
// constants
////

// minimal number of inliers required to accept an image as registered
uint32_t minimal_RANSAC_solution = 12;

// SIFT-ratio value. Since we store squared distances, we need the squared value 0.7^2 = 0.49
float nn_ratio = 0.49f;

// the assumed minimal inlier ratio of RANSAC
float min_inlier = 0.2f;

// stop RANSAC if 60 seconds have passed
double ransac_max_time = 60.0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------

////
// Actual localization method
////

int main (int argc, char **argv)
{
  ////
  // get the parameter
  uint32_t nb_clusters = (uint32_t) atoi( argv[1] );
  std::string cluster_file( argv[2] );
  std::string vw_assignments( argv[3] );
  int mode = atoi( argv[4] );
  if ( mode < 0 || mode > 2 )
  {
    std::cerr << " ERROR: unknown mode " << mode << std::endl;
    return -1;
  }


  Eigen::Matrix<float, 128, 128, Eigen::RowMajor> projection_matrix;
  std::ifstream ifs_projection_matrix(argv[5], std::ios::in);
  if (!ifs_projection_matrix.is_open()) {
    std::cerr << "ERROR: Cannot read the projection "
              << "matrix from " << argv[5] << std::endl;
    return -1;
  }

  std::ofstream ofs(argv[6], std::ios::out);

  // Loads the projection matrix.
  for (int i = 0; i < 128; ++i) {
    for (int j = 0; j < 128; ++j) {
      ifs_projection_matrix >> projection_matrix(i, j);
    }
  }

  ifs_projection_matrix.close();


  ////
  // load the visual words and their tree
  visual_words_handler vw_handler;
  vw_handler.set_nb_trees( 1 );
  vw_handler.set_nb_visual_words( nb_clusters );
  vw_handler.set_branching( 10 );

  vw_handler.set_method(std::string("flann"));
  //vw_handler.set_flann_type(std::string("randomkd"));
  vw_handler.set_flann_type(std::string("hkmeans"));
  if ( !vw_handler.create_flann_search_index( cluster_file ) )
  {
    std::cout << " ERROR: Could not load the cluster centers from " << cluster_file << std::endl;;
    return -1;
  }
  std::cout << "  done " << std::endl;


  ////
  // load the assignments for the visual words

  std::cout << "* Loading and parsing the assignments ... " << std::endl;

  // store the 3D positions of the 3D points
  std::vector< Eigen::Vector3f > points3D;

  // store the descriptors in a vector simply by concatenating their entries
  // depending on the mode, either unsigned char entries or floating point entries are used
  std::vector< unsigned char > all_descriptors;
  std::vector< float > all_descriptors_float;

  // for every visual word, store a vector of (3D point id, descriptor id) pairs, where the 3D point id
  // is the index of the 3D point in points3D and the descriptor id is the position of the first entry of the corresponding
  // descriptor in all_descriptors / all_descriptors_float
  std::vector< std::vector< std::pair< uint32_t, uint32_t > > > vw_points_descriptors(nb_clusters);

  // store per visual word the number of (point, descriptor) pairs store in it
  std::vector< uint32_t > nb_points_per_vw(nb_clusters, 0);

  // number of non-empty visual words, the number of 3D points and the total number of descriptors
  uint32_t nb_non_empty_vw, nb_3D_points, nb_descriptors;

  for ( uint32_t i = 0; i < nb_clusters; ++i )
    vw_points_descriptors[i].clear();

  // load the assignments from a file generated by compute_desc_assignments
  {
    std::ifstream ifs( vw_assignments.c_str(), std::ios::in | std::ios::binary  );

    if ( !ifs )
    {
      std::cerr << " ERROR: Cannot read the visual word assignments " << vw_assignments << std::endl;
      return -1;
    }

    uint32_t nb_clusts;
    ifs.read(( char* ) &nb_3D_points, sizeof( uint32_t ) );
    ifs.read(( char* ) &nb_clusts, sizeof( uint32_t ) );
    ifs.read(( char* ) &nb_non_empty_vw, sizeof( uint32_t ) );
    ifs.read(( char* ) &nb_descriptors, sizeof( uint32_t ) );

    //for sf_spec_100k_RSIFT_voc_branch300_MEAN.bin
    //nb_descriptors = 101355949;

    if ( nb_clusts != nb_clusters )
      std::cerr << " WARNING: Number of clusters differs! " << nb_clusts << " " << nb_clusters << std::endl;

    std::cout << "  Number of non-empty clusters: " << nb_non_empty_vw << " number of points : " << nb_3D_points << " number of descriptors: " << nb_descriptors << std::endl;

    // read the 3D points and their visibility polygons
    points3D.resize(nb_3D_points);
    if ( mode == 0 || mode == 2 )
      all_descriptors.resize(uint64_t(nb_descriptors) * 128, -1 );
    else
      all_descriptors_float.resize(uint64_t(nb_descriptors) * 128 );


    // load the points
    float *point_data = new float[3];
    for ( uint32_t i = 0; i < nb_3D_points; ++i )
    {
      ifs.read(( char* ) point_data, 3 * sizeof( float ) );
      for ( int j = 0; j < 3; ++j )
        points3D[i][j] = point_data[j];

      //std::cout << points3D[i][0] << " " << points3D[i][1] << " " << points3D[i][2] << std::endl;
    }
    delete [] point_data;

    // load the descriptors
    uint64_t index = 0;
    for ( uint32_t i = 0; i < nb_descriptors; ++i, index += sift_dim )
    {
      // std::cout << i << std::endl;
      for ( uint64_t j = 0; j < sift_dim; ++j )
      {
        if ( mode == 0 || mode == 2 ){
          ifs.read(( char* ) &all_descriptors[index + j], sizeof( unsigned char ) );
        }
        else
          ifs.read(( char* ) &all_descriptors_float[index + j], sizeof( float ) );
      }


      // bool found_an_empty = false;
      // for ( uint64_t j = 0; j < sift_dim; ++j )
      // {
      //   if(all_descriptors[index + j] < 0){
      //     found_an_empty = true;
      //     break;
      //   }
      // }
      // if(found_an_empty){
      //   std::cout << desc_nb << " is empty " << std::endl;
      //   empty_pt++;
      // }
      // if(empty_pt>100){
      //   break;
      // }
      // desc_nb++;
    }

    std::cout << "done loading descriptors" << std::endl;

    // now we load the assignments of the pairs (point_id, descriptor_id) to the visual words
    for ( uint32_t i = 0; i < nb_clusters; ++i )
    {
      uint32_t id, nb_pairs;
      ifs.read(( char* ) &id, sizeof( uint32_t ) );
      ifs.read(( char* ) &nb_pairs, sizeof( uint32_t ) );
      vw_points_descriptors[id].resize( nb_pairs );
      nb_points_per_vw[id] = nb_pairs;
      // ofs << nb_pairs << std::endl;
      for ( uint32_t j = 0; j < nb_pairs; ++j )
      {
        ifs.read(( char* ) &vw_points_descriptors[id][j].first, sizeof( uint32_t ) );
        ifs.read(( char* ) &vw_points_descriptors[id][j].second, sizeof( uint32_t ) );
      }
    //  std::cout << id << " " << nb_pairs << std::endl;
    }
    ifs.close();
    std::cout << "  done loading and parsing the assignments " << std::endl;
  }



  std::cout << "there are total " << all_descriptors.size() / 128 << " features" << std::endl;
  // for(uint32_t j = 16837338 * 128; j < 16837338 * 128 + 128; j++)
  // {
  //   std::cout << float(all_descriptors[j]) << std::endl;
  // }
  // return 0;

  /*this part is the version before iccv, lots of memeory used*/
  // //now we get all the assignments of all visual words, for each visual word, we compute the hamming thresholds
  // Eigen::Matrix<float, 64, Eigen::Dynamic> he_thresholds;
  // //this should be the same size of visual clusters size.
  // he_thresholds.resize(64, nb_clusters);
  // //now project the descriptors in each visual word into hamming space.
  // //this is the vector to store all projection results for each visual word
  // std::vector<std::vector<std::vector<float> > > entries_per_word(nb_clusters);
  // for (int i = 0; i < nb_clusters; ++i) {
  //   entries_per_word[i].resize(64);
  //   for (int j = 0; j < 64; ++j) entries_per_word[i][j].clear();
  // }

  // std::cout << "finish setting entries per word" << std::endl;

  // for (int i = 0; i < nb_clusters; ++i) {
  //   int in_word_nb =  nb_points_per_vw[i];
  //   if (in_word_nb == 0) {
  //     // std::cout << " WARNING: FOUND EMPTY WORD " << i << std::endl;
  //     he_thresholds.col(i) = Eigen::Matrix<float, 64, 1>::Zero();
  //   }
  //   else {
  //     for (int j = 0; j < in_word_nb; j++)
  //     {
  //       Eigen::Matrix<float, 128, 1> sift;
  //       //assign the corresponding sift feature
  //       uint64_t cur_desc = vw_points_descriptors[i][j].second;
  //       for (int k = 0; k < 128; k++)
  //       {
  //         uint64_t cur_feature_index = cur_desc * 128 + uint64_t(k);
  //         //std::cout << float(all_descriptors[cur_feature_index]) << std::endl;
  //         sift(k, 0) = all_descriptors[cur_feature_index];
  //       }
  //       //do the hamming projection
  //       Eigen::Matrix<float, 64, 1> proj_sift = projection_matrix * sift;
  //       for (int k = 0; k < 64; ++k) {
  //         entries_per_word[i][k].push_back(proj_sift[k]);
  //       }
  //     }
  //     //compute the thresholds
  //     const int median_element = in_word_nb / 2;
  //     for (int k = 0; k < 64; ++k) {
  //       std::nth_element(entries_per_word[i][k].begin(),
  //                        entries_per_word[i][k].begin() + median_element,
  //                        entries_per_word[i][k].end());
  //       he_thresholds(k, i) = entries_per_word[i][k][median_element];
  //     }

  //   }
  //   if(i % 1000 == 0)
  //     std::cout << "thresholding " << i << " / " << nb_clusters << std::endl;
  // }

  // std::cout << "finish getting the hamming thresholds" << std::endl;
  /*this part is the version before iccv, lots of memeory used*/



  /*this part if for iccv*/
  //now we get all the assignments of all visual words, for each visual word, we compute the hamming thresholds
  Eigen::Matrix<float, 128, Eigen::Dynamic> he_thresholds;
  //this should be the same size of visual clusters size.
  he_thresholds.resize(128, nb_clusters);
  //now project the descriptors in each visual word into hamming space.
  //this is the vector to store all projection results for each visual word


  std::cout << "finish setting entries per word" << std::endl;

  for (int i = 0; i < nb_clusters; ++i) {
    //output the first assignment for test
    if(i == 0){
      std::cout << "the first cluster id " << i << " nb pairs " << nb_points_per_vw[0] << std::endl;
    }

    std::vector<std::vector<std::vector<float> > > entries_per_word(1);
    entries_per_word[0].resize(128);
      for (int j = 0; j < 128; ++j) entries_per_word[0][j].clear();
    

    int in_word_nb =  nb_points_per_vw[i];
    if (in_word_nb == 0) {
      std::cout << " WARNING: FOUND EMPTY WORD " << i << std::endl;
      he_thresholds.col(i) = Eigen::Matrix<float, 128, 1>::Zero();
    }
    else {
      for (int j = 0; j < in_word_nb; j++)
      {
        //std::cout << i << " " << in_word_nb << std::endl;
        Eigen::Matrix<float, 128, 1> sift;
        //assign the corresponding sift feature
        uint64_t cur_desc = vw_points_descriptors[i][j].second;
        for (int k = 0; k < 128; k++)
        {
          uint64_t cur_feature_index = cur_desc * 128 + uint64_t(k);
          //std::cout << float(all_descriptors[cur_feature_index]) << std::endl;
          sift(k, 0) = all_descriptors[cur_feature_index];
        }
        //do the hamming projection
        Eigen::Matrix<float, 128, 1> proj_sift = projection_matrix * sift;
        for (int k = 0; k < 128; ++k) {
          entries_per_word[0][k].push_back(proj_sift[k]);
        }
      }
      //compute the thresholds
      const int median_element = in_word_nb / 2;
      for (int k = 0; k < 128; ++k) {
        std::nth_element(entries_per_word[0][k].begin(),
                         entries_per_word[0][k].begin() + median_element,
                         entries_per_word[0][k].end());
        he_thresholds(k, i) = entries_per_word[0][k][median_element];
      }

    }
    if (i % 10000 == 0)
      std::cout << "thresholding " << i << " / " << nb_clusters << std::endl;
  }

  std::cout << "finish getting the hamming thresholds" << std::endl;


  //after performing thresholding. compute the hamming embedding for each descriptors in database.
  //for each visual word, encode the pair of binary descriptors and point index.
  std::vector <std::string> all_binary_descriptors;
  all_binary_descriptors.resize(nb_descriptors);
  for (int i = 0; i < nb_clusters; ++i) {
    int in_word_nb =  nb_points_per_vw[i];
    if (in_word_nb == 0) {
      // std::cout << " Do not compute the binary descriptors " << i << std::endl;
    }
    else {
      for (int j = 0; j < in_word_nb; j++)
      {
        Eigen::Matrix<float, 128, 1> sift;
        //assign the corresponding sift feature
        uint64_t cur_desc = vw_points_descriptors[i][j].second;
        for (int k = 0; k < 128; k++)
        {
          uint64_t cur_feature_index = cur_desc * 128 + uint64_t(k);
          sift(k, 0) = all_descriptors[cur_feature_index];
        }
        //do the hamming projection
        Eigen::Matrix<float, 128, 1> proj_sift = projection_matrix * sift;
        //for each dimension, calculate the binary with median hamming thresholds
        std::bitset<128> binary_descriptor;
        for (int k = 0 ; k < 128; k++)
        {
          binary_descriptor[k] = proj_sift[k] > he_thresholds(k, i);
        }
        all_binary_descriptors[cur_desc] = binary_descriptor.to_string();
      }
    }
    if (i % 1000 == 0)
      std::cout << "transferring " << i << " / " << nb_clusters << std::endl;
  }

  std::cout << "finish transferring to binary" << std::endl;



  if (!ofs.is_open()) {
    std::cerr << "ERROR: Cannot write to " << argv[6] << std::endl;
    return -1;
  }

  ofs << nb_3D_points << " " << nb_clusters << " " << nb_non_empty_vw << " " << nb_descriptors << std::endl;
  ofs << nb_clusters << " 128 128" << std::endl;
  for (int i = 0; i < nb_clusters; ++i) {
    for (int j = 0; j < 128; ++j) {
      ofs << std::setprecision(16) << he_thresholds(j, i) << " ";
    }
    ofs << std::endl;
  }

  std::cout << "Finish writting the hamming thresholds" << std::endl;
  for (int i = 0; i < 128; ++i) {
    for (int j = 0; j < 128; ++j) {
      ofs << std::setprecision(16) << projection_matrix(i, j) << " ";
    }
    ofs << std::endl;
  }
  std::cout << "Finish writting the projection matrix" << std::endl;


  //write the binary descriptors
  for (int i = 0; i < all_binary_descriptors.size(); i++)
  {
    // std::cout << i << std::endl;
    if (i % 1000000 == 0) {
      std::cout << i << std::endl;
    }
    ofs << all_binary_descriptors[i] << std::endl;
  }
  all_binary_descriptors.clear();
  all_descriptors.clear();
  all_descriptors_float.clear();
  std::cout << "Finish writting the binary descriptors" << std::endl;

  //write the assignments.
  for (int i = 0; i < nb_clusters; ++i) {
    ofs << i << " " << nb_points_per_vw[i] << std::endl;
    for (int j = 0; j < nb_points_per_vw[i]; j++)
    {
      ofs << vw_points_descriptors[i][j].first << " " << vw_points_descriptors[i][j].second << " ";
    }
    ofs << std::endl;
    if (i % 1000 == 0)
      std::cout << "writting assigments " << i << " / " << nb_clusters << std::endl;
  }
  ofs.close();
  return 0;
}

