#define __STDC_LIMIT_MACROS

// C++ includes
#include <bitset>
#include <vector>
#include <set>
#include <map>
#include <iostream>
#include <sstream>
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
#include "sfm/parse_bundler.hh"

// stopwatch
#include "timer.hh"

#include "exif_reader/exif_reader.hh"

const uint64_t sift_dim = 128;

class Spatial_Bin
{
public:
	int w_idx;
	int h_idx;
	std::vector< std::pair< double, int > > bin_desc_dist;
	int contained;
	float local_ratio;
	int quota;
};

bool compare_score(const std::pair< double, int > &a, const std::pair< double, int > &b)
{
	return (a.first > b.first);
}

bool cmp_dist( const std::pair< int, float >& a, const std::pair< int, float >& b )
{
	return ( a.second < b.second );
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------

int main (int argc, char **argv)
{
	if ( argc < 15 )
	{
		std::cout << "______________________________________________________________________________________________________________________________________" << std::endl;
		std::cout << " - Usage: cascaded_parallel_filtering_aachenDayNight                                                                                - " << std::endl;
		std::cout << " - Generate two set of 2D-3D matches for final geometry-wise disambiguation for Aachen Day-Night dataset                            - " << std::endl;
		std::cout << " - In this code, the dimension of hamming binary signatures is set to 64                                                            - " << std::endl;
		std::cout << " - Parameters:                                                                                                                      - " << std::endl;
		std::cout << " -  argv[1]: The list of query images                                                                                               - " << std::endl;
		std::cout << " -  argv[2]: The number of visual words                                                                                             - " << std::endl;
		std::cout << " -  argv[3]: The detailed visual vocabulary                                                                                         - " << std::endl;
		std::cout << " -  argv[4]: The quantization results and hamming thresholds and projection matrix                                                  - " << std::endl;
		std::cout << " -  argv[5]: The number of branching factor in hierarchical k-means tree                                                            - " << std::endl;
		std::cout << " -  argv[6]: The .info file, note that we do not load the SIFT/RootSIFT feature descriptors in this file                            - " << std::endl;
		std::cout << " -  argv[7]: The Hamming distance threshold                                                                                         - " << std::endl;
		std::cout << " -  argv[8]: The threshold of vote number                                                                                           - " << std::endl;
		std::cout << " -  argv[9]: The k for retriving top-k database images                                                                              - " << std::endl;
		std::cout << " -  argv[10]: The k1 for retriving top-k1 database images (k1>k)                                                                    - " << std::endl;
		std::cout << " -  argv[11]: The bilateral ratio test used in the query image side                                                                 - " << std::endl;
		std::cout << " -  argv[12]: The match score threshold                                                                                             - " << std::endl;
		std::cout << " -  argv[13] and argv[14]: The output 2D-3D matches                                                                                 - " << std::endl;
		std::cout << " -  argv[13] stores the 2D positions and argv[14] stores the 3D positions                                                           - " << std::endl;
		std::cout << " -  (first 2D-3D matches for computing the auxliary camera pose, then 2D-3D matches that serve as Visibility-wise match pool )      - " << std::endl;
		std::cout << "______________________________________________________________________________________________________________________________________" << std::endl;
		return -1;
	}
	std::string keylist( argv[1] );
	uint32_t nb_clusters = (uint32_t) atoi( argv[2] );
	std::string cluster_file( argv[3] );
	std::string hamming_results( argv[4] );
	int nb_branching = atoi( argv[5] );
	parse_bundler parser;
	std::string bundle_file( argv[6] );
	//Read the data from file in .info format. Note that we exclude loading the original SIFT/RootSIFT descriptors
	//Instead, we use the binary descriptors in hamming_results(arg[4])
	parser.load_from_binary_nokey( bundle_file.c_str(), 1 );
	size_t hamming_dist_threshold = (size_t) atoi( argv[7] );
	int valid_corrs_threshold =  atoi( argv[8] );
	int top_rank_k = atoi( argv[9] );
	int top_rank_k1 = atoi( argv[10] );
	double ratio_test_threshold = atof(argv[11]);
	double score_threshold = atof(argv[12]);

	std::string pos_2d( argv[13] );
	// create and open the output file
	std::ofstream ofs_2d( pos_2d.c_str(), std::ios::out );
	std::string pos_3d( argv[14] );
	// create and open the output file
	std::ofstream ofs_3d( pos_3d.c_str(), std::ios::out );
	uint32_t nb_cameras = parser.get_number_of_cameras();
	uint32_t nb_points_bundler = parser.get_number_of_points();
	std::vector< feature_3D_info >& feature_infos = parser.get_feature_infos();
	std::vector< bundler_camera >& camera_infos = parser.get_cameras();
	visual_words_handler vw_handler;
	vw_handler.set_nb_trees( 1 );
	vw_handler.set_nb_visual_words( nb_clusters );
	vw_handler.set_branching( nb_branching );

	vw_handler.set_method(std::string("flann"));
	vw_handler.set_flann_type(std::string("hkmeans"));
	if ( !vw_handler.create_flann_search_index( cluster_file ) )
	{
		std::cout << " ERROR: Could not load the cluster centers from " << cluster_file << std::endl;;
		return -1;
	}
	std::cout << "  done " << std::endl;

	// load the assignments for the visual words and binary descriptors.

	std::cout << "* Loading and parsing the assignments ... " << std::endl;
	std::vector < std::bitset<64> > all_binary_descriptors;
	// store the 3D positions of the 3D points

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
	std::ifstream ifs( hamming_results.c_str(), std::ios::in  );
	std::cout << "read file from " << hamming_results << std::endl;

	uint32_t nb_clusts;
	ifs >> nb_3D_points >> nb_clusts >> nb_non_empty_vw >> nb_descriptors;
	std::cout << " num of descriptors " << nb_descriptors << std::endl;

	//read the he thresholds
	Eigen::Matrix<float, 64, Eigen::Dynamic> he_thresholds;
	Eigen::Matrix<float, 64, 128, Eigen::RowMajor> projection_matrix;

	int num_words; int num_dimensions; int num_bits;
	ifs >> num_words >> num_dimensions >> num_bits;
	//read the hamming thresholds of visual words.
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
		if (nb_pairs <= 5)
			nb_small_clusters++;
		if (nb_pairs == 0)
			empty_clusters++;
		int pt_id; int desc_id;
		for (int j = 0; j < nb_pairs ; ++j)
		{
			ifs >> pt_id >> desc_id;
			vw_points_descriptors[id][j].first = pt_id;
			vw_points_descriptors[id][j].second = desc_id;
		}
	}
	ifs.close();
	std::cout << "  done loading assignments, small clusters " << nb_small_clusters
	          << " empty clusters " << empty_clusters  << std::endl;


	// now load all the filenames of the query images
	// read the query image list provided by Aachen Day-Night dataset.
	// remember to replace all .jpg to .key
	std::vector< std::string > key_filenames;
	std::vector< float > input_width;
	std::vector< float > input_height;
	std::vector< float > focal_length;
	std::vector< float > cx;
	std::vector< float > cy;
	std::vector< float > radial;
	key_filenames.clear();
	input_width.clear();
	input_height.clear();
	focal_length.clear();
	cx.clear(); cy.clear(); radial.clear();

	std::ifstream ifs_key( keylist.c_str(), std::ios::in );
	std::string tmp_string;
	std::string tmp_string2;
	float tmp_width, tmp_height, tmp_focal, tmp_cx, tmp_cy, tmp_radial;

	while ( !ifs_key.eof() )
	{
		tmp_string = "";
		tmp_string2 = " ";
		ifs_key >> tmp_string >> tmp_string2 >> tmp_width >> tmp_height >> tmp_focal
		        >> tmp_cx >> tmp_cy >> tmp_radial;
		if ( !tmp_string.empty() )
		{
			key_filenames.push_back(tmp_string);
			input_width.push_back(tmp_width);
			input_height.push_back(tmp_height);
			focal_length.push_back(tmp_focal);
			cx.push_back(tmp_cx);
			cy.push_back(tmp_cy);
			radial.push_back(tmp_radial);
		}

	}
	ifs_key.close();
	std::cout << " done loading " << key_filenames.size() << " keyfile names " << std::endl;


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
	std::vector< std::pair< uint32_t, uint32_t > > corrs;
	std::vector< std::pair< double, uint32_t > > corrs_score;
	std::vector< std::pair< double, uint32_t > > corrs_ratio_test;
	std::vector< std::pair< int, float > > desc_dist;

	//first 2d keypoint id, second 3d point id
	for ( uint32_t i = 0; i < nb_keyfiles; ++i, nb_query += 1.0)
	{
		corrs.clear();
		desc_dist.clear();
		corrs_score.clear();
		corrs_ratio_test.clear();
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
			descriptors.clear();
			keypoints.clear();
			continue;
		}

		for ( uint32_t j = 0; j < nb_loaded_keypoints; ++j )
		{
			keypoints[j].x -= (img_width - 1.0) / 2.0f;
			keypoints[j].y = (img_height - 1.0) / 2.0f - keypoints[j].y;
		}

		//we use 4x4 bins to divide the query image
		//the bottom left corresponds to bin 0 and the top right corresponds to bin 15
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
				bin[j * w_cell + k].bin_desc_dist.clear();
				bin[j * w_cell + k].w_idx = j;
				bin[j * w_cell + k].h_idx = k;
				bin[j * w_cell + k].contained = 0;
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
			}
		}

		std::vector< std::vector<int> > query_set(nb_loaded_keypoints);
		for ( uint32_t j = 0; j < nb_loaded_keypoints; ++j )
			query_set[j].clear();

		for (int j = 0; j < feature_infos.size(); j++)
		{
			feature_infos[j].matched_query.clear();
		}

		Timer all_timer;
		all_timer.Init();
		all_timer.Start();

		Timer time_;
		time_.Init();
		time_.Start();

		if ( computed_visual_words.size() < nb_loaded_keypoints )
			computed_visual_words.resize( nb_loaded_keypoints );

		vw_handler.set_nb_paths( 1);
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

			int match_in_query = feature_infos[cur_3d_id].matched_query.size();
			for (int k = 0; k < match_in_query; k++)
			{
				cur_avg_in_query_distance += (double)feature_infos[cur_3d_id].matched_query[k];
			}
			double ratio_test_in_query = (double)desc_dist[j].second * (double)match_in_query *
			                             (double)match_in_query / cur_avg_in_query_distance;

			double hamming_ratio = cur_avg_feature_distance / (double)(desc_dist[j].second + 1);

			double oper;
			if (desc_dist[j].second <= 8)
			{
				oper = 0.5f;
			}
			else {
				oper =  (double)desc_dist[j].second / 16.0f;
			}

			double score = ( hamming_ratio * exp(-1.0f * oper * oper)) / (oper * oper);
			corrs_score.push_back(std::make_pair(score, j));
			corrs_ratio_test.push_back(std::make_pair(ratio_test_in_query, j));
		}

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
		int qualified_corrs_nb = 0;
		std::map<std::string, bool> checkingmap;
		for (int j = 0; j < corrs.size(); j++)
		{
			if (corrs_ratio_test[j].first <= 1.0f / ratio_test_threshold)
			{
				int cur_3d_pt = corrs[j].second;
				int cur_2d_pt = corrs[j].first;

				qualified_corrs_nb++;
				for (int k = 0; k < feature_infos[cur_3d_pt].view_list.size(); k++)
				{
					bool find_multiple = false;
					int cur_img = feature_infos[cur_3d_pt].view_list[k].camera;
					for (int vt = 0; vt < camera_infos[cur_img].vote_list.size(); vt++)
					{
						if (corrs[camera_infos[cur_img].vote_list[vt]].first == cur_2d_pt)
						{
							find_multiple = true;
						}
					}
					if (!find_multiple)
						camera_infos[cur_img].vote_list.push_back(j);
				}
			}
		}
		std::cout << "there are " << qualified_corrs_nb << " matches passing the ratio test " << std::endl;


		std::vector< std::pair< double, uint32_t > > camera_rank;
		camera_rank.clear();
		//calculate the term frequency for each image
		for (int j = 0; j < camera_infos.size(); j++)
		{
			if (camera_infos[j].vote_list.size() > 0)
			{
				for (int k = 0; k < camera_infos[j].vote_list.size(); k++)
				{
					if (corrs_score[camera_infos[j].vote_list[k]].first >= 0.8)
					{
						camera_infos[j].probability += corrs_score[camera_infos[j].vote_list[k]].first;
						camera_infos[j].valid_corrs_nb++;
						camera_infos[j].avg_hamming_distance += desc_dist[camera_infos[j].vote_list[k]].second;
					}
				}
				double nb_pt_per_db = camera_infos[j].point_list.size();
				camera_infos[j].probability /= sqrt(nb_pt_per_db);
				double vote_pt_per_db = camera_infos[j].vote_list.size();
				camera_infos[j].avg_hamming_distance /= vote_pt_per_db;
			}
			else
				camera_infos[j].probability = 0;

			if (camera_infos[j].valid_corrs_nb >= valid_corrs_threshold)
				camera_rank.push_back(std::make_pair(camera_infos[j].probability, j));
		}

		std::sort(camera_rank.begin(), camera_rank.end(), compare_score);
		time_.Stop();
		avrg_voting_time = avrg_voting_time * nb_query / (nb_query + 1.0) + time_.GetElapsedTime() / (nb_query + 1.0);
		std::cout << "average voting time " << avrg_voting_time << "s" << std::endl;

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
		std::cout << "the corrs score size " << corrs_score.size() << std::endl;
		std::vector< std::pair< double, uint32_t > > new_corrs_score = corrs_score;

		for (int j = 0; j < camera_rank.size(); j++)
		{
			if (j >= top_rank_k)
				break;
			int top_cam = camera_rank[j].second;
			int confident_pt_nb = 0;
			int augment_pt_nb = 0;
			for (int k = 0; k < camera_infos[top_cam].vote_list.size(); k++)
			{
				if (corrs_score[camera_infos[top_cam].vote_list[k]].first >= score_threshold)
					confident_pt_nb++;
				else
					augment_pt_nb++;
			}
			double update_step = 0.5 *  log(1 + (double)confident_pt_nb / (double)augment_pt_nb) * score_threshold;
			for (int k = 0; k < camera_infos[top_cam].vote_list.size(); k++)
			{
				if (corrs_score[camera_infos[top_cam].vote_list[k]].first < score_threshold)
					new_corrs_score[camera_infos[top_cam].vote_list[k]].first += update_step;
			}
		}
		std::vector< std::pair< double, int > > corrs_in_top_img;
		corrs_in_top_img.clear();
		for (int j = 0; j < camera_rank.size(); j++)
		{
			if (j >= top_rank_k)
				break;
			int top_cam = camera_rank[j].second;
			for (int k = 0; k < camera_infos[top_cam].vote_list.size(); k++)
			{
				if (!picked[camera_infos[top_cam].vote_list[k]])
				{
					if (corrs_score[camera_infos[top_cam].vote_list[k]].first >= score_threshold)
					{
						corrs_in_top_img.push_back(std::make_pair(corrs_score[camera_infos[top_cam].vote_list[k]].first, camera_infos[top_cam].vote_list[k]));
						picked[camera_infos[top_cam].vote_list[k]] = true;
					}
				}
			}
		}
		//sort the corrs
		std::sort(corrs_in_top_img.begin(), corrs_in_top_img.end(), compare_score);

		//do the local voting
		//reset the pick list
		for (int j = 0; j < corrs.size(); j++)
		{
			picked[j] = false;
		}
		int nb_top_corrs = 0;
		for (int j = 0; j < camera_rank.size(); j++)
		{
			if (j >= top_rank_k)
				break;
			int top_cam = camera_rank[j].second;
			for (int k = 0; k < camera_infos[top_cam].vote_list.size(); k++)
			{
				if (!picked[camera_infos[top_cam].vote_list[k]])
				{
					int cur_id = camera_infos[top_cam].vote_list[k];
					if (new_corrs_score[cur_id].first >= score_threshold)
					{
						picked[cur_id] = true;
						int w_idx = (int)((keypoints[corrs[cur_id].first].x + half_w)  / w_cell_size );
						int h_idx = (int)((keypoints[corrs[cur_id].first].y + half_h) / h_cell_size);
						bin[w_idx * w_cell + h_idx].bin_desc_dist.push_back(std::make_pair(new_corrs_score[cur_id].first, cur_id));
						nb_top_corrs++;
					}
				}
			}

		}

		std::cout << "there are total " << nb_top_corrs << " corrs in top image" << std::endl;

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

		for (int j = 0; j < corrs_in_top_img.size(); j++ )
		{
			int cur_id = corrs_in_top_img[j].second;
			int w_idx = (int)( (keypoints[corrs[cur_id].first].x + half_w)  / w_cell_size );
			int h_idx = (int)((keypoints[corrs[cur_id].first].y + half_h) / h_cell_size);

			if (bin[w_idx * w_cell + h_idx].contained < bin[w_idx * w_cell + h_idx].quota )
			{
				if (corrs_in_top_img[j].first >= score_threshold)
				{
					chosen_pt.push_back(cur_id);
					occupied[cur_id] = true;
					bin[w_idx * w_cell + h_idx].contained++;
				}
			}
		}

		std::cout << "done pick the global best corrs " << chosen_pt.size() << std::endl;

		int spatial_augmentation_quota = 1.33 * chosen_pt.size();


		for (int j = 0; j < bin.size(); j++)
		{
			if (chosen_pt.size() >= spatial_augmentation_quota)
				break;
			std::sort(bin[j].bin_desc_dist.begin(), bin[j].bin_desc_dist.end(), compare_score);
			for (int k = 0; k < bin[j].bin_desc_dist.size(); k++)
			{
				if (bin[j].contained >= bin[j].quota)
					break;
				if (!occupied[bin[j].bin_desc_dist[k].second])
				{
					if (bin[j].bin_desc_dist[k].first >= score_threshold)
					{
						chosen_pt.push_back(bin[j].bin_desc_dist[k].second);
						occupied[bin[j].bin_desc_dist[k].second] = true;
						bin[j].contained++;
					}
				}
			}
		}
		std::cout << "after spatial augmention " << chosen_pt.size() << std::endl;

		time_.Init();
		time_.Start();

		for (int j = 0; j < camera_rank.size(); j++)
		{
			if (j >= top_rank_k1)
				break;
			int top_cam = camera_rank[j].second;
			for (int k = 0; k < camera_infos[top_cam].vote_list.size(); k++)
			{
				if (!potential_picked[camera_infos[top_cam].vote_list[k]])
				{
					potential_chosen_pt.push_back(camera_infos[top_cam].vote_list[k]);
					potential_picked[camera_infos[top_cam].vote_list[k]] = true;
				}
			}
		}
		std::cout << "potential chosen point size " << potential_chosen_pt.size() << std::endl;

		time_.Stop();
		avrg_final_pick_time = avrg_final_pick_time * nb_query / (nb_query + 1.0) + time_.GetElapsedTime() / (nb_query + 1.0);
		std::cout << "average final pick time " << avrg_final_pick_time << "s" << std::endl;

		all_timer.Stop();
		avrg_selection_time = avrg_selection_time * nb_query / (nb_query + 1.0) + all_timer.GetElapsedTime() / (nb_query + 1.0);
		std::cout << "average selection time " << avrg_selection_time << "s" << std::endl;
		//remove the directory of filename
		const size_t last_slash_idx = jpg_filename.find_last_of("\\/");
		if (std::string::npos != last_slash_idx)
		{
			jpg_filename.erase(0, last_slash_idx + 1);
		}
		ofs_2d << i << " " << chosen_pt.size() << " " << jpg_filename << " "
		       << input_width[i] << " " << input_height[i] << " " << focal_length[i] <<
		       " " << cx[i] << " " << cy[i] << " " << radial[i] << std::endl;
		ofs_3d << i << " " << chosen_pt.size() << std::endl;

		for (int j = 0; j < chosen_pt.size(); j++ )
		{
			ofs_2d << keypoints[corrs[chosen_pt[j]].first].x << " " << keypoints[corrs[chosen_pt[j]].first].y << std::endl;
			ofs_3d << std::setprecision(16) << feature_infos[corrs[chosen_pt[j]].second].point.x << " "
			       << std::setprecision(16) << feature_infos[corrs[chosen_pt[j]].second].point.y << " "
			       << std::setprecision(16) << feature_infos[corrs[chosen_pt[j]].second].point.z << std::endl;

		}
		ofs_2d << i << " " << potential_chosen_pt.size() << std::endl;
		ofs_3d << i << " " << potential_chosen_pt.size() << std::endl;
		for (int j = 0; j < potential_chosen_pt.size(); j++ )
		{
			//std::cout << feature_infos[corrs[chosen_pt[j]].second].matched_query.size() << " ";
			ofs_2d << keypoints[corrs[potential_chosen_pt[j]].first].x << " "
			       << keypoints[corrs[potential_chosen_pt[j]].first].y << std::endl;
			ofs_3d << std::setprecision(16) << feature_infos[corrs[potential_chosen_pt[j]].second].point.x << " "
			       << std::setprecision(16) << feature_infos[corrs[potential_chosen_pt[j]].second].point.y << " "
			       << std::setprecision(16) << feature_infos[corrs[potential_chosen_pt[j]].second].point.z << std::endl;
		}

		for ( uint32_t j = 0; j < nb_loaded_keypoints; ++j )
		{
			if ( descriptors[j] != 0 )
				delete [] descriptors[j];
			descriptors[j] = 0;
		}
		descriptors.clear();
		keypoints.clear();
		for (int j = 0; j < feature_infos.size(); j++)
		{
			feature_infos[j].matched_query.clear();
		}
		corrs.clear();
		bin.clear();
		query_set.clear();
		desc_dist.clear();
		corrs_score.clear();
		new_corrs_score.clear();
		corrs_ratio_test.clear();

	}
	ofs_2d.close();
	ofs_3d.close();
	return 0;
}








