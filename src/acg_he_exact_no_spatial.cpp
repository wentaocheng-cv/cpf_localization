


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
#include <math.h>
#include <stdio.h>


// includes for classes dealing with SIFT-features
#include "features/SIFT_loader.hh"
#include "features/visual_words_handler.hh"

// stopwatch
#include "timer.hh"

// math functionality
#include "math/projmatrix.hh"
#include "math/matrix3x3.hh"

// RANSAC
#include "RANSAC.hh"

// exif reader to get the width and height out
// of the exif tags of an image
#include "exif_reader/exif_reader.hh"

// simple vector class for 3D points
#include <OpenMesh/Core/Geometry/VectorT.hh>

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


class Spatial_Bin
{
public:
	int w_idx;
	int h_idx;
	std::vector< std::pair< int, int > > bin_corrs;
	std::vector< std::pair< double, int > > bin_desc_dist;
	std::vector< int > global_corrs_id;
	int contained;
	float corrs_size;
	float local_ratio;
	int quota;
};






bool compare_prob(const bundler_camera &a, const bundler_camera &b)
{
	return (a.probability > b.probability);
}


bool compare_score(const std::pair< double, int > &a, const std::pair< double, int > &b)
{
	return (a.first > b.first);
}

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

bool cmp_dist( const std::pair< int, float >& a, const std::pair< int, float >& b )
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
	std::string keylist( argv[1] );
	size_t nb_trees = (size_t) atoi( argv[2] );
	uint32_t nb_clusters = (uint32_t) atoi( argv[3] );
	std::string cluster_file( argv[4] );
	std::string vw_assignments( argv[5] );
	std::string results( argv[6] );
	// create and open the output file
	std::ofstream ofs_details( results.c_str(), std::ios::out );

	if ( !ofs_details.is_open() )
	{
		std::cerr << " Could not write results to " << results << std::endl;
		return 1;
	}
	parse_bundler parser;
	std::string bundle_file( argv[7] );
	if ( !parser.parse_data( bundle_file.c_str(), 0 ) )
	{
		std::cerr << " ERROR: Could not parse the bundler file " << bundle_file << std::endl;
		return -1;
	}

	std::string pos_2d( argv[8] );
	// create and open the output file
	std::ofstream ofs_2d( pos_2d.c_str(), std::ios::out );
	std::string pos_3d( argv[9] );
	// create and open the output file
	std::ofstream ofs_3d( pos_3d.c_str(), std::ios::out );
	uint32_t nb_cameras = parser.get_number_of_cameras();
	uint32_t nb_points_bundler = parser.get_number_of_points();
	std::vector< feature_3D_info >& feature_infos = parser.get_feature_infos();
	std::vector< bundler_camera >& camera_infos = parser.get_cameras();

	for (int i = 0; i < feature_infos.size(); i++)
	{
		feature_infos[i].matched_query.clear();
	}

	int top_rank_k = atoi( argv[10] );
	int top_rank_k1 = atoi( argv[11] );
	std::cout << "top rank image number " << top_rank_k  << " and " << top_rank_k1 << std::endl;
	double ratio_test_threshold = atof(argv[12]);
	std::cout << "hamming ratio threshold in the query image " << ratio_test_threshold << std::endl;
	double score_threshold = atof(argv[13]);
	std::cout << "per match score threshold is " << score_threshold << std::endl;
	size_t hamming_dist_threshold = (size_t) atoi( argv[14] );
	std::cout << "hamming distance threshold is " << hamming_dist_threshold << std::endl;
	int valid_corrs_threshold =  atoi( argv[15] );
	std::cout << "valid corrs threshold is " << valid_corrs_threshold << std::endl;
	int write_or_not = atoi( argv[16] );
	std::cout << "write or not " << write_or_not << std::endl;
	////
	// load the visual words and their tree
	//create the flann tree of vocabulary trees.
	visual_words_handler vw_handler;
	vw_handler.set_nb_trees( nb_trees );
	vw_handler.set_nb_visual_words( nb_clusters );
	vw_handler.set_branching( 10 );

	vw_handler.set_method(std::string("flann"));
	vw_handler.set_flann_type(std::string("hkmeans"));
	if ( !vw_handler.create_flann_search_index( cluster_file ) )
	{
		std::cout << " ERROR: Could not load the cluster centers from " << cluster_file << std::endl;;
		return -1;
	}
	std::cout << "  done " << std::endl;


	////
	// load the assignments for the visual words and binary descriptors.

	std::cout << "* Loading and parsing the assignments ... " << std::endl;
	std::vector < std::bitset<64> > all_binary_descriptors;
	// store the 3D positions of the 3D points
	std::vector< OpenMesh::Vec3f > points3D;

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
	std::ifstream ifs( vw_assignments.c_str(), std::ios::in  );
	std::cout << "read file from " << vw_assignments << std::endl;

	uint32_t nb_clusts;
	ifs >> nb_3D_points >> nb_clusts >> nb_non_empty_vw >> nb_descriptors;

	//read the he thresholds
	Eigen::Matrix<float, 64, Eigen::Dynamic> he_thresholds;
	Eigen::Matrix<float, 64, 128, Eigen::RowMajor> projection_matrix;

	int num_words; int num_dimensions; int num_bits;
	ifs >> num_words >> num_dimensions >> num_bits;
	he_thresholds.resize(num_bits, num_words);
	for (int i = 0; i < num_words; ++i) {
		for (int j = 0; j < num_bits; ++j) {
			ifs >> he_thresholds(j, i);
		}
	}
	//read projection matrix
	for (int i = 0; i < num_bits; ++i) {
		for (int j = 0; j < num_dimensions; ++j) {
			ifs >> projection_matrix(i, j);
		}
	}

	//read the binary descriptors as uint_64
	all_binary_descriptors.resize(nb_descriptors);
	for (int i = 0; i < nb_descriptors; ++i) {
		uint64_t tmp_desc;
		ifs >> tmp_desc;
		all_binary_descriptors[i] = std::bitset<64>(tmp_desc);
	}

	//read assignments;
	int nb_small_clusters = 0;
	int empty_clusters = 0;
	for (int i = 0; i < nb_clusters; ++i) {
		int id; int nb_pairs;
		ifs >> id >> nb_pairs;
		vw_points_descriptors[id].resize( nb_pairs );
		nb_points_per_vw[id] = nb_pairs;
		// std::cout << nb_pairs << std::endl;
		if (nb_pairs <= 5)
			nb_small_clusters++;
		if (nb_pairs == 0)
			empty_clusters++;
		int pt_id; int desc_id;
		// std::vector< int > db_occurence(nb_cameras);
		// for ( uint32_t j = 0; j < nb_cameras; ++j )
		//   db_occurence[j] = 0;
		for (int j = 0; j < nb_pairs ; ++j)
		{
			ifs >> pt_id >> desc_id;
			vw_points_descriptors[id][j].first = pt_id;
			vw_points_descriptors[id][j].second = desc_id;
			// //for each visual word, count the occurence of each db image
			// for(int k = 0; k < feature_infos[pt_id].view_list.size(); k ++)
			// {
			//   db_occurence[feature_infos[pt_id].view_list[k].camera]++;
			// }
		}
		// for(int j = 0; j < nb_cameras; j++)
		// {
		//   if(db_occurence[j] > 0)
		//     std::cout << db_occurence[j] << " ";
		// }
		// std::cout << std::endl;
	}
	ifs.close();
	std::cout << "  done loading assignments, small clusters " << nb_small_clusters
	          << " empty clusters " << empty_clusters  << std::endl;


	// now load all the filenames of the query images
	std::vector< std::string > key_filenames;
	key_filenames.clear();
	{
		std::ifstream ifs( keylist.c_str(), std::ios::in );
		std::string tmp_string;

		while ( !ifs.eof() )
		{
			tmp_string = "";
			ifs >> tmp_string;
			if ( !tmp_string.empty() )
				key_filenames.push_back(tmp_string);
		}
		ifs.close();
		std::cout << " done loading " << key_filenames.size() << " keyfile names " << std::endl;
	}

	uint32_t nb_keyfiles = key_filenames.size();

	// do the actual localization
	// store all assignments of 2D features to visual words in one large vector (resized if necessary)
	// preallocated for speed
	std::vector< uint32_t > computed_visual_words( 50000, 0 );
	double nb_query = 0.0;

	double avrg_selection_time = 0.0;
	double avrg_matching_time = 0.0;
	double avrg_final_pick_time = 0.0;
	double avrg_voting_time = 0.0;
	double avrg_vw_time = 0.0;
	// compute nearest neighbors
	// we do a single case distinction wether the database consists of unsigned char descriptors or floating point descriptors
	// while resulting in ugly code, it should improve run-time since we do not need to make the distinction multiple time
	std::vector< std::pair< uint32_t, uint32_t > > corrs;
	std::vector< std::pair< double, uint32_t > > corrs_score;
	std::vector< std::pair< double, uint32_t > > corrs_ratio_test;
	std::vector< std::pair< int, float > > desc_dist;
	//to store which vw the corrs come from
	std::vector< std::pair< int, int > > vw_size;
	//clear the match query list
	for (int j = 0; j < feature_infos.size(); j++)
	{
		feature_infos[j].matched_query.clear();
	}
	corrs.clear();
	desc_dist.clear();
	vw_size.clear();
	corrs_score.clear();
	corrs_ratio_test.clear();
	//first 2d keypoint id, second 3d point id
	for ( uint32_t i = 0; i < nb_keyfiles; ++i, nb_query += 1.0)
	{
		// load the features
		SIFT_loader key_loader;
		key_loader.load_features( key_filenames[i].c_str(), LOWE );

		std::vector< unsigned char* >& descriptors = key_loader.get_descriptors();
		std::vector< SIFT_keypoint >& keypoints = key_loader.get_keypoints();

		uint32_t nb_loaded_keypoints = (uint32_t) keypoints.size();


		// center the keypoints around the center of the image
		// first we need to get the dimensions of the image which we obtain from its exif tag
		int img_width, img_height;
		std::string jpg_filename( key_filenames[i] );
		jpg_filename.replace( jpg_filename.size() - 3, 3, "jpg");
		exif_reader::open_exif( jpg_filename.c_str() );
		img_width = exif_reader::get_image_width();
		img_height = exif_reader::get_image_height();
		exif_reader::close_exif();

		double max_width = 0; double max_height = 0;
		for ( uint32_t j = 0; j < nb_loaded_keypoints; ++j )
		{
			if (keypoints[j].x > max_width)
				max_width = keypoints[j].x;
			if (keypoints[j].y > max_height)
				max_height = keypoints[j].y;
		}

		std::cout << i << " " << img_width << " " << img_height << " " << nb_loaded_keypoints << std::endl;
		std::cout << "max width " << max_width << " max height " << max_height << std::endl;

		if (max_width > img_width || max_height > img_height)
		{
			std::cout << "query image " << i << " has a wrong ----------------------------------------- exif info" << std::endl;
			for ( uint32_t j = 0; j < nb_loaded_keypoints; ++j )
			{
				if ( descriptors[j] != 0 )
					delete [] descriptors[j];
				descriptors[j] = 0;
			}
			//std::cout << "image " << i << " SIFT " << nb_loaded_keypoints << " he corrs " << nb_potential_corrs << std::endl;
			descriptors.clear();
			keypoints.clear();
			continue;
		}


		for ( uint32_t j = 0; j < nb_loaded_keypoints; ++j )
		{
			keypoints[j].x -= (img_width - 1.0) / 2.0f;
			keypoints[j].y = (img_height - 1.0) / 2.0f - keypoints[j].y;
		}

		//first we put all corrs into 16 bins.
		const int h_cell = 4;
		const int w_cell = 4;
		float half_w = 0.5 * float(img_width - 1);
		float half_h = 0.5 * float(img_height - 1);
		float w_cell_size = float(img_width) / float(w_cell);
		float h_cell_size = float(img_height) / float(h_cell);
		std::vector<Spatial_Bin> bin;
		bin.clear();
		bin.resize(h_cell * w_cell);
		for (int j = 0; j < w_cell; j++  )
		{
			for (int k = 0; k < h_cell; k++)
			{
				bin[j * w_cell + k].bin_corrs.clear();
				bin[j * w_cell + k].bin_desc_dist.clear();
				bin[j * w_cell + k].w_idx = j;
				bin[j * w_cell + k].h_idx = k;
				bin[j * w_cell + k].contained = 0;
				bin[j * w_cell + k].corrs_size = 0;
				bin[j * w_cell + k].local_ratio = 0;
				bin[j * w_cell + k].quota = 0;

			}
		}

		//load the SIFT descriptors into a large eigen matrix
		Eigen::Matrix<float, 128, Eigen::Dynamic> query_sift;
		query_sift.resize(128, nb_loaded_keypoints);
		for (int j = 0; j < nb_loaded_keypoints; ++j)
		{
			for (int k = 0; k < 128; ++k)
			{
				query_sift(k, j) = (float)descriptors[j][k];
				//query_sift(k, j) = sqrt((float)descriptors[j][k]);
			}
		}

		std::vector< std::vector<int> > query_set(nb_loaded_keypoints);
		for ( uint32_t j = 0; j < nb_loaded_keypoints; ++j )
			query_set[j].clear();


		Timer all_timer;
		all_timer.Init();
		all_timer.Start();

		Timer time_;
		time_.Init();
		time_.Start();

		if ( computed_visual_words.size() < nb_loaded_keypoints )
			computed_visual_words.resize( nb_loaded_keypoints );

		vw_handler.set_nb_paths( 10 );
		vw_handler.assign_visual_words_ucharv( descriptors, nb_loaded_keypoints, computed_visual_words );

		time_.Stop();
		avrg_vw_time = avrg_vw_time * nb_query / (nb_query + 1.0) + time_.GetElapsedTime() / (nb_query + 1.0);
		std::cout << "average assign vw time " << avrg_vw_time << "s" << std::endl;


		time_.Init();
		time_.Start();

		int corrs_index = 0;
		for ( size_t j = 0; j < nb_loaded_keypoints; ++j )
		{
			//get the assigned visual word index.
			uint32_t assignment = uint32_t( computed_visual_words[j] );
			//first, project the SIFT to hamming space.
			Eigen::Matrix<float, 64, 1> proj_sift = projection_matrix * query_sift.col(j);
			//generate the binary descriptor
			std::bitset<64> binary_descriptor;
			for (int k = 0 ; k < 64; k++)
			{
				binary_descriptor[k] = proj_sift[k] > he_thresholds(k, assignment);
			}
			//in the visual words, compute the hamming distance to each db binary descriptors.
			int per_vw_size = vw_points_descriptors[assignment].size();
			// if(per_vw_size <= 10)
			//   matched_to_small_vw++;
			if (per_vw_size > 0)
			{
				for (int m = 0; m < per_vw_size; ++m)
				{
					int binary_id = vw_points_descriptors[assignment][m].second;
					size_t hamming_dist = (binary_descriptor ^ all_binary_descriptors[binary_id]).count();
					if (hamming_dist <= hamming_dist_threshold)
					{
						query_set[j].push_back(hamming_dist);
						feature_infos[vw_points_descriptors[assignment][m].first].matched_query.push_back(hamming_dist);
						desc_dist.push_back(std::make_pair(corrs_index , hamming_dist));
						vw_size.push_back(std::make_pair(corrs_index , assignment));
						corrs.push_back(std::make_pair( j, vw_points_descriptors[assignment][m].first ));
						corrs_index++;
					}
				}
			}
		}
		std::cout << "query " << i << " << corrs number ---------------- " << corrs.size() << std::endl;
		time_.Stop();
		avrg_matching_time = avrg_matching_time * nb_query / (nb_query + 1.0) + time_.GetElapsedTime() / (nb_query + 1.0);
		std::cout << "average hamming feature matching time " << avrg_matching_time << "s" << std::endl;



		time_.Init();
		time_.Start();
		//compute the score of each correspondence.
		for (int j = 0; j < corrs.size(); j++)
		{
			int cur_2d_id = corrs[j].first;
			int cur_3d_id = corrs[j].second;
			double cur_avg_feature_distance = 0.0f;
			double cur_avg_in_query_distance = 0.0f;
			int q_set_size = query_set[cur_2d_id].size();
			for (int k = 0; k < q_set_size; k++)
			{
				cur_avg_feature_distance += (double)query_set[cur_2d_id][k];
			}
			cur_avg_feature_distance /= (double) q_set_size;
			// std::cout << " avg " << cur_avg_feature_distance << std::endl;

			int match_in_query = feature_infos[cur_3d_id].matched_query.size();
			for (int k = 0; k < match_in_query; k++)
			{
				cur_avg_in_query_distance += (double)feature_infos[cur_3d_id].matched_query[k];
			}
			//cur_avg_in_query_distance /= (double) match_in_query;

			double ratio_test_in_query = (double)desc_dist[j].second * (double)match_in_query *
			                             (double)match_in_query / cur_avg_in_query_distance;

			//double ratio_test_in_query = (double)desc_dist[j].second/ cur_avg_in_query_distance ;
			double hamming_ratio = cur_avg_feature_distance / (double)(desc_dist[j].second + 1);


			// //scheme 1
			// double oper;
			// if (desc_dist[j].second <= 8)
			// {
			// 	oper = 0.5f;
			// }
			// else {
			// 	oper =  (double)desc_dist[j].second / 16.0f;
			// }
			// double score = ( hamming_ratio * exp(-1.0f * oper * oper))  ;


			// //scheme 2
			// double oper;
			// oper =  (double)desc_dist[j].second / 16.0f;
			// double score = (hamming_ratio * exp(-1.0f * oper * oper)) / (oper * oper);

			//scheme 3
			double oper;
			if (desc_dist[j].second <= 8)
			{
				oper = 8.0f / 16.0f;
			}
			else {
				oper =  (double)desc_dist[j].second / 16.0f;
			}
			double score = ( hamming_ratio * exp(-1.0f * oper * oper)) / (oper * oper)  ;
			//double score = ( hamming_ratio * exp(-1.0f * oper * oper)) / oper   ;
			//double score = ( hamming_ratio * exp(-1.0f * oper * oper))  ;


			//the old function
			// double oper =  (double)desc_dist[j].second / 16.0f;
			// double score = exp(-1.0f * oper * oper);

			corrs_score.push_back(std::make_pair(score, j));
			corrs_ratio_test.push_back(std::make_pair(ratio_test_in_query, j));
		}
		//std::cout << "!" << std::endl;

		//do the voting using the corresponding corrs
		//clear the voting list
		for (int j = 0; j < nb_cameras; j++)
		{
			camera_infos[j].vote_list.clear();
			camera_infos[j].probability = 0;
			camera_infos[j].valid_corrs_nb = 0;
			camera_infos[j].avg_hamming_distance = 0;
		}

		//do the voting
		int pass_ratio_test = 0;
		for (int j = 0; j < corrs.size(); j++)
		{
			if (corrs_ratio_test[j].first <= ratio_test_threshold)
			{
				pass_ratio_test++;
				int cur_3d_pt = corrs[j].second;
				int cur_2d_pt = corrs[j].first;
				for (int k = 0; k < feature_infos[cur_3d_pt].view_list.size(); k++)
				{
					// bool find_multiple = false;
					// int cur_img = feature_infos[cur_3d_pt].view_list[k].camera;
					// for (int vt = 0; vt < camera_infos[cur_img].vote_list.size(); vt++)
					// {
					// 	if (corrs[camera_infos[cur_img].vote_list[vt]].first == cur_2d_pt)
					// 	{
					// 		find_multiple = true;
					// 	}
					// }
					// //store the index of the corrs for voting each image
					// if (!find_multiple)
					// 	camera_infos[cur_img].vote_list.push_back(j);


					int cur_img = feature_infos[cur_3d_pt].view_list[k].camera;
					camera_infos[cur_img].vote_list.push_back(j);
				}

			}
		}

		std::cout << "there are " << pass_ratio_test << " out of " << corrs.size() << " passing the ratio test" << std::endl;

		// //sf dataset style
		// for (int j = 0; j < corrs.size(); j++)
		// {

		// 	if (corrs_ratio_test[j].first <= ratio_test_threshold)
		// 	{
		// 		int cur_3d_pt = corrs[j].second;
		// 		int cur_2d_pt = corrs[j].first;
		// 		int cur_vw_id = vw_size[j].second;
		// 		for (int k = 0; k < feature_infos[cur_3d_pt].view_list.size(); k++)
		// 		{
		// 			bool find_multiple = false;
		// 			bool need_replace = false;
		// 			int replace_place = 0;
		// 			int cur_img = feature_infos[cur_3d_pt].view_list[k].camera;
		// 			for (int vt = 0; vt < camera_infos[cur_img].vote_list.size(); vt++)
		// 			{
		// 				//if we found that a query vote multiple times to the same databast image. we only preserve the
		// 				//most similar one
		// 				if (corrs[camera_infos[cur_img].vote_list[vt]].first == cur_2d_pt)
		// 				{
		// 					find_multiple = true;
		// 					if (corrs_score[camera_infos[cur_img].vote_list[vt]].first < corrs_score[j].first)
		// 					{
		// 						need_replace = true;
		// 						replace_place = vt;
		// 					}

		// 				}
		// 				//if we found that a point vote for multiple times for the query image.
		// 				//we also preserve the most similar one.
		// 				if (corrs[camera_infos[cur_img].vote_list[vt]].second == cur_3d_pt)
		// 				{
		// 					find_multiple = true;
		// 					if (corrs_score[camera_infos[cur_img].vote_list[vt]].first < corrs_score[j].first)
		// 					{
		// 						need_replace = true;
		// 						replace_place = vt;
		// 					}
		// 				}
		// 				//if we found that a match corresponds to the same visual word. we penalize the score. but store them
		// 				// not remove them.
		// 				if (vw_size[camera_infos[cur_img].vote_list[vt]].second == cur_vw_id)
		// 				{
		// 					find_multiple = true;
		// 				}
		// 			}

		// 			//store the index of the corrs for voting each image
		// 			if (!find_multiple)
		// 			{
		// 				camera_infos[cur_img].vote_list.push_back(j);
		// 				camera_infos[cur_img].identical_visual_world_nb++;;
		// 			}
		// 			if (find_multiple && need_replace)
		// 				camera_infos[cur_img].vote_list[replace_place] = j;
		// 		}
		// 	}
		// }

		//std::cout << "!!" << std::endl;
		std::vector< std::pair< double, uint32_t > > camera_rank;
		camera_rank.clear();
		//calculate the term frequency for each image
		for (int j = 0; j < camera_infos.size(); j++)
		{
			if (camera_infos[j].vote_list.size() > 0)
			{
				for (int k = 0; k < camera_infos[j].vote_list.size(); k++)
				{
					if (corrs_score[camera_infos[j].vote_list[k]].first >= score_threshold)
					{
						int cur_2d_pt = corrs[camera_infos[j].vote_list[k]].first;
						//double many_point = double(query_set[cur_2d_pt].size());
						camera_infos[j].valid_corrs_nb++;
						camera_infos[j].probability += corrs_score[camera_infos[j].vote_list[k]].first;
					}

					camera_infos[j].avg_hamming_distance += desc_dist[camera_infos[j].vote_list[k]].second;
				}
				double nb_pt_per_db = camera_infos[j].point_list.size();
				//double identical_vw_nb = camera_infos[j].identical_visual_world_nb;
				double vote_pt_per_db = camera_infos[j].vote_list.size();

				camera_infos[j].probability /= sqrt(nb_pt_per_db);
				//camera_infos[j].probability /= nb_pt_per_db;
				//camera_infos[j].probability *= identical_vw_nb / vote_pt_per_db;
				//here we penalize the same visual wors occuring

				camera_infos[j].avg_hamming_distance /= vote_pt_per_db;
			}
			else
				camera_infos[j].probability = 0;

			if (camera_infos[j].valid_corrs_nb >= valid_corrs_threshold)
				camera_rank.push_back(std::make_pair(camera_infos[j].probability, j));

			// camera_rank.push_back(std::make_pair(camera_infos[j].probability, j));
		}
		//std::cout << "!!!" << std::endl;

		std::sort(camera_rank.begin(), camera_rank.end(), compare_score);
		time_.Stop();
		avrg_voting_time = avrg_voting_time * nb_query / (nb_query + 1.0) + time_.GetElapsedTime() / (nb_query + 1.0);
		std::cout << "average voting time " << avrg_voting_time << "s" << std::endl;

		// std::vector< bundler_camera > tmp_camera_infos;
		// tmp_camera_infos.clear();
		// for (int j = 0; j < camera_infos.size(); j++)
		// {
		// 	if (camera_infos[j].vote_list.size() >= 3)
		// 		tmp_camera_infos.push_back(camera_infos[j]);
		// }

		// std::sort(tmp_camera_infos.begin(), tmp_camera_infos.end(), compare_prob);



		//return the points in the top ranked images.
		//for a corrs, as long as it is visible in the top images. return it.
		std::vector< int > chosen_pt;
		chosen_pt.clear();
		std::vector<bool> picked;
		picked.clear();
		picked.resize(corrs.size());
		std::vector<bool> potential_picked;
		potential_picked.clear();
		potential_picked.resize(corrs.size());
		std::vector< int > potential_chosen_pt;
		potential_chosen_pt.clear();
		for (int j = 0; j < corrs.size(); j++)
		{
			picked[j] = false;
		}
		for (int j = 0; j < corrs.size(); j++)
		{
			potential_picked[j] = false;
		}

		//define 16 bins,quantize all corrs into 16 bins
		std::vector<bool> occupied;
		occupied.clear();
		occupied.resize(corrs.size());
		for (int j = 0; j < occupied.size(); j++)
		{
			occupied[j] = false;
		}

		//score updating
		std::vector< std::pair< double, uint32_t > > new_corrs_score = corrs_score;
		double image_distance_threshold = 24;

		for (int j = 0; j < camera_rank.size(); j++)
		{
			if (j >= top_rank_k)
				break;
			int top_cam = camera_rank[j].second;
			int confident_pt_nb = 0;
			int augment_pt_nb = 0;
			if (camera_infos[top_cam].avg_hamming_distance <= image_distance_threshold)
			{
				for (int k = 0; k < camera_infos[top_cam].vote_list.size(); k++)
				{
					if (corrs_score[camera_infos[top_cam].vote_list[k]].first >= score_threshold)
						confident_pt_nb++;
					else
						augment_pt_nb++;
				}
				// std::cout << "rank " << j << " " << confident_pt_nb << ""
				//double update_step =  (double)confident_pt_nb * score_threshold * 0.5 / (double)augment_pt_nb;
				double update_step = 0.5 *  log(1 + (double)confident_pt_nb / (double)augment_pt_nb) * score_threshold;
				//double update_step =  (double)confident_pt_nb * 0.1;
				for (int k = 0; k < camera_infos[top_cam].vote_list.size(); k++)
				{
					if (corrs_score[camera_infos[top_cam].vote_list[k]].first < score_threshold)
						new_corrs_score[camera_infos[top_cam].vote_list[k]].first += update_step;
				}
			}
		}
		//first do the best image picking
		int nb_selected_best = 0;
		std::vector< std::pair< double, int > > corrs_in_top_img;
		corrs_in_top_img.clear();
		for (int j = 0; j < camera_rank.size(); j++)
		{
			nb_selected_best++;
			if (nb_selected_best > top_rank_k)
				break;
			int top_cam = camera_rank[j].second;
			if (camera_infos[top_cam].avg_hamming_distance <= image_distance_threshold)
			{
				for (int k = 0; k < camera_infos[top_cam].vote_list.size(); k++)
				{
					if (!picked[camera_infos[top_cam].vote_list[k]])
					{
						if (corrs_score[camera_infos[top_cam].vote_list[k]].first >= score_threshold)
						{
							//chosen_pt.push_back(tmp_camera_infos[j].vote_list[k]);
							//here we use the score which only encode the local feature information
							corrs_in_top_img.push_back(std::make_pair(corrs_score[camera_infos[top_cam].vote_list[k]].first, camera_infos[top_cam].vote_list[k]));
							picked[camera_infos[top_cam].vote_list[k]] = true;
							new_corrs_score[camera_infos[top_cam].vote_list[k]].first = 0;
							//int w_idx = (int)((keypoints[corrs[camera_infos[top_cam].vote_list[k]].first].x + half_w)  / w_cell_size );
							//int h_idx = (int)((keypoints[corrs[camera_infos[top_cam].vote_list[k]].first].y + half_h) / h_cell_size);
							//bin[w_idx * w_cell + h_idx].bin_desc_dist.push_back(std::make_pair(corrs_score[camera_infos[top_cam].vote_list[k]].first, camera_infos[top_cam].vote_list[k]));
						}
					}
				}
			}
		}
		//sort the corrs
		std::sort(corrs_in_top_img.begin(), corrs_in_top_img.end(), compare_score);
		std::cout << "high confidence corrs " << corrs_in_top_img.size() << std::endl;

		// int complementary_size = 0.33 * corrs_in_top_img.size();
		// std::cout << "can pick " << complementary_size << " complementary match more" << std::endl;



		std::vector< std::pair< double, int > > comple_corrs_in_top_img;
		comple_corrs_in_top_img.clear();
		// //do the local voting
		// //reset the pick list
		for (int j = 0; j < corrs.size(); j++)
		{
			picked[j] = false;
		}
		//compute the quota for each bin
		int nb_selected_img = 0;
		int nb_top_corrs = 0;
		for (int j = 0; j < camera_rank.size(); j++)
		{
			nb_selected_img++;
			if (nb_selected_img > top_rank_k)
				break;
			// if (nb_top_corrs >= complementary_size)
			// 	break;
			int top_cam = camera_rank[j].second;
			if (camera_infos[top_cam].avg_hamming_distance <= image_distance_threshold)
			{
				for (int k = 0; k < camera_infos[top_cam].vote_list.size(); k++)
				{
					if (!picked[camera_infos[top_cam].vote_list[k]])
					{
						int cur_id = camera_infos[top_cam].vote_list[k];
						if (new_corrs_score[cur_id].first >= score_threshold)
						{
							//throw the top 100 corrs into each bin.
							picked[cur_id] = true;
							comple_corrs_in_top_img.push_back(std::make_pair(new_corrs_score[camera_infos[top_cam].vote_list[k]].first, camera_infos[top_cam].vote_list[k]));
							nb_top_corrs++;
						}
						// if (nb_top_corrs >= complementary_size)
						// 	break;
					}
				}
			}
		}
		std::cout << "there are total " << nb_top_corrs << " corrs in top image" << std::endl;
		std::sort(comple_corrs_in_top_img.begin(), comple_corrs_in_top_img.end(), compare_score);

		float root_bin_sum = 0;
		for (int j = 0; j < bin.size(); j++)
		{
			root_bin_sum += pow(float(bin[j].bin_desc_dist.size()), 0.5);
		}

		for (int j = 0; j < bin.size(); j++)
		{
			bin[j].local_ratio = pow(float(bin[j].bin_desc_dist.size()), 0.5) / root_bin_sum;
			bin[j].quota = int(100 * bin[j].local_ratio);
		}


		// std::cout << "start pick global best corrs" << std::endl;

		// for (int j = 0; j < corrs_in_top_img.size(); j++ )
		// {
		// 	int cur_id = corrs_in_top_img[j].second;
		// 	int w_idx = (int)( (keypoints[corrs[cur_id].first].x + half_w)  / w_cell_size );
		// 	int h_idx = (int)((keypoints[corrs[cur_id].first].y + half_h) / h_cell_size);
		// 	// std::cout << "bin " << w_idx * w_cell + h_idx << std::endl;
		// 	if (corrs_in_top_img[j].first >= 0.8)
		// 	{
		// 		chosen_pt.push_back(cur_id);
		// 		occupied[cur_id] = true;
		// 		bin[w_idx * w_cell + h_idx].contained++;
		// 		// std::cout << corrs_in_top_img[j].first << std::endl;
		// 	}
		// }

		// std::cout << "done pick the global best corrs " << chosen_pt.size() << std::endl;


		std::cout << "start pick global best corrs" << std::endl;
		int nb_picked = 0;
		for (int j = 0; j < corrs_in_top_img.size(); j++ )
		{
			int cur_id = corrs_in_top_img[j].second;
			if(nb_picked > 75)
				break;
			// std::cout << "bin " << w_idx * w_cell + h_idx << std::endl;
			if (corrs_in_top_img[j].first >= score_threshold)
			{
				chosen_pt.push_back(cur_id);
				nb_picked++;
			}
		}
		std::cout << "done pick the global best corrs " << chosen_pt.size() << std::endl;

		// for (int j = 0 ; j < bin.size(); j++)
		// {
		// 	std::cout << "bin " << j << " contain " << bin[j].contained
		// 	          << " ratio " << bin[j].local_ratio << " quota " <<
		// 	          bin[j].quota << std::endl;
		// }

		int spatial_augmentation_quota = 1.33 * chosen_pt.size();
		//int spatial_augmentation_quota = chosen_pt.size();
		//std::cout << "can pick to " << spatial_augmentation_quota << " points" << std::endl;
		for (int j = 0; j < comple_corrs_in_top_img.size(); j++ )
		{
			int cur_id = corrs_in_top_img[j].second;
			if(chosen_pt.size() > spatial_augmentation_quota)
				break;
			// std::cout << "bin " << w_idx * w_cell + h_idx << std::endl;
			if (comple_corrs_in_top_img[j].first >= score_threshold)
			{
				chosen_pt.push_back(cur_id);
			}
		}
		std::cout << "done pick the global best corrs " << chosen_pt.size() << std::endl;


		std::cout << "after spatial augmention " << chosen_pt.size() << std::endl;

		// for (int j = 0 ; j < bin.size(); j++)
		// {
		// 	std::cout << "bin " << j << " contain " << bin[j].contained << " quota " <<
		// 	          bin[j].quota << std::endl;
		// }

		time_.Init();
		time_.Start();



		//for writing the voted images information
		ofs_details << "query " << i  << " corrs " << corrs.size() << std::endl;
		int cur_write_img = 0;
		for (int j = 0; j < camera_rank.size(); j++)
		{
			cur_write_img++;
			if (cur_write_img > top_rank_k1)
				break;
			float avg_desc_dist = 0;
			int top_cam = camera_rank[j].second;
			if (write_or_not != 0)
			{
				ofs_details << "image -------------------- " << camera_infos[top_cam].id
				            << " " << camera_infos[top_cam].point_list.size() << " "
				            << camera_infos[top_cam].probability << " rank " << j << std::endl;
			}



			for (int k = 0; k < camera_infos[top_cam].vote_list.size(); k++)
			{
				if (write_or_not != 0)
				{
					ofs_details <<  desc_dist[camera_infos[top_cam].vote_list[k]].second
					            << " " << corrs[camera_infos[top_cam].vote_list[k]].first
					            << " " << feature_infos[corrs[camera_infos[top_cam].vote_list[k]].second].matched_query.size()
					            << " " << query_set[corrs[camera_infos[top_cam].vote_list[k]].first].size() <<
					            " " << corrs_score[camera_infos[top_cam].vote_list[k]].first
					            << " " << new_corrs_score[camera_infos[top_cam].vote_list[k]].first
					            << " " << camera_infos[top_cam].vote_list[k]
					            << " " << vw_size[camera_infos[top_cam].vote_list[k]].second << std::endl ;
				}
				//avg_desc_dist += desc_dist[tmp_camera_infos[j].vote_list[k]].second;
				if (!potential_picked[camera_infos[top_cam].vote_list[k]])
				{
					// if (new_corrs_score[tmp_camera_infos[j].vote_list[k]].first >= score_threshold)
					// {
					potential_chosen_pt.push_back(camera_infos[top_cam].vote_list[k]);
					potential_picked[camera_infos[top_cam].vote_list[k]] = true;
					//}
				}
			}

			//ofs_details << tmp_camera_infos[j].id << " average distance " << tmp_camera_infos[j].avg_hamming_distance << std::endl;

		}
		std::cout << "potential chosen point size " << potential_chosen_pt.size() << std::endl;



		time_.Stop();
		avrg_final_pick_time = avrg_final_pick_time * nb_query / (nb_query + 1.0) + time_.GetElapsedTime() / (nb_query + 1.0);
		std::cout << "average final pick time " << avrg_final_pick_time << "s" << std::endl;

		all_timer.Stop();
		avrg_selection_time = avrg_selection_time * nb_query / (nb_query + 1.0) + all_timer.GetElapsedTime() / (nb_query + 1.0);
		std::cout << "average selection time " << avrg_selection_time << "s" << std::endl;
		ofs_2d << i << " " << chosen_pt.size() << std::endl;
		ofs_3d << i << " " << chosen_pt.size() << std::endl;
		for (int j = 0; j < chosen_pt.size(); j++ )
		{
			ofs_2d << keypoints[corrs[chosen_pt[j]].first].x << " " <<  keypoints[corrs[chosen_pt[j]].first].y << std::endl;
			ofs_3d << feature_infos[corrs[chosen_pt[j]].second].point.x << " "
			       << feature_infos[corrs[chosen_pt[j]].second].point.y << " "
			       << feature_infos[corrs[chosen_pt[j]].second].point.z << std::endl;

		}

		ofs_2d << i << " " << potential_chosen_pt.size() << std::endl;
		ofs_3d << i << " " << potential_chosen_pt.size() << std::endl;
		for (int j = 0; j < potential_chosen_pt.size(); j++ )
		{
			//std::cout << feature_infos[corrs[chosen_pt[j]].second].matched_query.size() << " ";
			ofs_2d << keypoints[corrs[potential_chosen_pt[j]].first].x << " " <<  keypoints[corrs[potential_chosen_pt[j]].first].y << std::endl;
			ofs_3d << feature_infos[corrs[potential_chosen_pt[j]].second].point.x << " "
			       << feature_infos[corrs[potential_chosen_pt[j]].second].point.y << " "
			       << feature_infos[corrs[potential_chosen_pt[j]].second].point.z << std::endl;
		}

		// do the pose verification using RANSAC
		// clean up
		for ( uint32_t j = 0; j < nb_loaded_keypoints; ++j )
		{
			if ( descriptors[j] != 0 )
				delete [] descriptors[j];
			descriptors[j] = 0;
		}
		//std::cout << "image " << i << " SIFT " << nb_loaded_keypoints << " he corrs " << nb_potential_corrs << std::endl;
		descriptors.clear();
		keypoints.clear();
		//clear the match query list
		for (int j = 0; j < feature_infos.size(); j++)
		{
			feature_infos[j].matched_query.clear();
		}
		corrs.clear();
		desc_dist.clear();
		vw_size.clear();
		corrs_score.clear();
		corrs_ratio_test.clear();

	}
	ofs_details.close();
	ofs_2d.close();
	ofs_3d.close();
	return 0;
}


