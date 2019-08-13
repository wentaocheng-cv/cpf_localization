/******************tip version of p4p ransac***********/

// #include <glog/logging.h>
// #include <gflags/gflags.h>
// #include <time.h>
// #include <theia/theia.h>
// #include <chrono>
// #include <string>
// #include <vector>
// #include <iostream>
// #include <fstream>
// using Eigen::Map;
// using Eigen::Matrix;
// using Eigen::Matrix3d;
// using Eigen::Vector3d;
// using Eigen::Vector2d;



// bool sortasce(const std::pair<int, int> &a,
//               const std::pair<int, int> &b)
// {
//   return (a.first > b.first);
// }

// bool compare_score(const std::pair< double, int > &a, const std::pair< double, int > &b)
// {
//   return (a.first > b.first);
// }

// int main(int argc, char *argv[]) {

//   //the file of input 2D-3D correspondences.
//   std::ifstream in_2d_pos( argv[1], std::ios::in );
//   std::ifstream in_3d_pos( argv[2], std::ios::in );
//   std::ifstream camera_truth( argv[3], std::ios::in );
//   std::string pos_2d( argv[4] );
//   std::ofstream ofs_2d( pos_2d.c_str(), std::ios::out );
//   std::string pos_3d( argv[5] );
//   std::ofstream ofs_3d( pos_3d.c_str(), std::ios::out );
//   std::string focal_length_file( argv[6] );
//   std::ofstream ofs_focal_length( focal_length_file.c_str(), std::ios::out );

//   std::vector<Eigen::Vector2d> points2D;
//   std::vector<Eigen::Vector3d> points3D;
//   std::vector<Eigen::Vector2d> full_points2D;
//   std::vector<Eigen::Vector3d> full_points3D;

//   std::vector<Eigen::Vector3d> camera_pos;
//   camera_pos.clear();
//   while (!camera_truth.eof())
//   {
//     double cam_x; double cam_y; double cam_z;
//     camera_truth >> cam_x >> cam_y >> cam_z;
//     Eigen::Vector3d cam_3d(cam_x, cam_y, cam_z);
//     camera_pos.push_back(cam_3d);
//   }


//   while (!in_2d_pos.eof() && !in_3d_pos.eof())
//   {
//     int cam_id; int corrs_nb;
//     in_2d_pos >> cam_id >> corrs_nb;
//     points2D.clear();
//     points2D.reserve(corrs_nb);
//     for (int i = 0; i < corrs_nb; i++)
//     {
//       double x_2d; double y_2d;
//       in_2d_pos >> x_2d >> y_2d;
//       Eigen::Vector2d insert_2d(x_2d, -1 * y_2d);
//       //Eigen::Vector2d insert_2d(x_2d,  y_2d);
//       points2D.push_back(insert_2d);
//     }

//     //read the full set of correspondences
//     int full_cam_id; int full_corrs_nb;
//     in_2d_pos >> full_cam_id >> full_corrs_nb;
//     full_points2D.clear();
//     full_points2D.reserve(full_corrs_nb);
//     for (int i = 0; i < full_corrs_nb; i++)
//     {
//       double full_x_2d; double full_y_2d;
//       in_2d_pos >> full_x_2d >> full_y_2d;
//       Eigen::Vector2d full_insert_2d(full_x_2d, -1 * full_y_2d);
//       //Eigen::Vector2d insert_2d(x_2d,  y_2d);
//       full_points2D.push_back(full_insert_2d);
//     }

//     // std::cout << "corrs " << cam_id << " " << corrs_nb  <<
//     //           " full " << full_cam_id << " " << full_corrs_nb << std::endl;

//     int cam_id_3d; int corrs_nb_3d;
//     in_3d_pos >> cam_id_3d >> corrs_nb_3d;
//     points3D.clear();
//     points3D.reserve(corrs_nb_3d);
//     for (int i = 0; i < corrs_nb_3d; i++)
//     {
//       double x_3d; double y_3d; double z_3d;
//       in_3d_pos >> x_3d >> y_3d >> z_3d;
//       Eigen::Vector3d insert_3d( x_3d, y_3d, z_3d);
//       points3D.push_back(insert_3d);
//     }

//     //read the full set of correspondences
//     int full_cam_id_3d; int full_corrs_nb_3d;
//     in_3d_pos >> full_cam_id_3d >> full_corrs_nb_3d;
//     full_points3D.clear();
//     full_points3D.reserve(full_corrs_nb_3d);
//     for (int i = 0; i < full_corrs_nb_3d; i++)
//     {
//       double full_x_3d; double full_y_3d; double full_z_3d;
//       in_3d_pos >> full_x_3d >> full_y_3d >> full_z_3d;
//       Eigen::Vector3d full_insert_3d( full_x_3d, full_y_3d, full_z_3d);
//       full_points3D.push_back(full_insert_3d);
//     }




//     // std::cout << "corrs " << cam_id_3d << " " << corrs_nb_3d  <<
//     //           " full " << full_cam_id_3d << " " << full_corrs_nb_3d << std::endl;
//     int num_correspondences = 0;
//     //check the consistency.
//     if (points3D.size() == points2D.size())
//     {
//       num_correspondences = points3D.size();
//     }

//     // for (int k = 0; k < num_correspondences; ++k)
//     // {

//     //   double color_r = ((double) rand() / (RAND_MAX));
//     //   double color_g = ((double) rand() / (RAND_MAX));
//     //   double color_b = ((double) rand() / (RAND_MAX));

//     //   std::cout << "plot(" << points2D[k][0] + 800.0 << "," <<
//     //             532.0 + points2D[k][1] << ",'.','color',[" << color_r << "," << color_g << "," << color_b << "],'MarkerSize',15)" << std::endl;


//     // }

//     //do the detailed ransac.

//     clock_t begin_timer;
//     clock_t end_timer;
//     Matrix<double, 3, 4> best_camera_matrix;
//     if (num_correspondences > 0)
//     {
//       begin_timer = clock();
//       int max_t = 1000;
//       std::vector<int> random_seqs;
//       random_seqs.resize(4 * max_t);
//       srand((int)time(0));
//       for (int i = 0; i < 4 * max_t; i++)
//       {
//         random_seqs[i] = rand() % num_correspondences;
//         // std::cout << random_seqs[i] << std::endl;
//       }
//       std::vector<Vector2d> sample_points_2D(4);
//       std::vector<Vector3d> sample_points_3D(4);

//       double pixel_threshold = 10.0f;


//       std::vector<Matrix<double, 3, 4> > all_ransac_solution;
//       all_ransac_solution.clear();
//       std::vector< std::pair< int, int > > all_inlier_nb;
//       all_inlier_nb.clear();
//       int all_inlier_nb_index = 0;
//       //int inlier_threshold = 11;
//       for (int  t = 0; t < max_t; t++)
//       {
//         sample_points_2D[0] = points2D[random_seqs[4 * t]];
//         sample_points_3D[0] = points3D[random_seqs[4 * t]];
//         sample_points_2D[1] = points2D[random_seqs[4 * t + 1]];
//         sample_points_3D[1] = points3D[random_seqs[4 * t + 1]];
//         sample_points_2D[2] = points2D[random_seqs[4 * t + 2]];
//         sample_points_3D[2] = points3D[random_seqs[4 * t + 2]];
//         sample_points_2D[3] = points2D[random_seqs[4 * t + 3]];
//         sample_points_3D[3] = points3D[random_seqs[4 * t + 3]];

//         //compute the pose using p4pf
//         std::vector<Matrix<double, 3, 4> > soln_projection;
//         int num_solns = theia::FourPointPoseAndFocalLength(
//                           sample_points_2D, sample_points_3D, &soln_projection);

//         int best_solution_id = 0;
//         int max_inliers = 0;
//         if (num_solns > 0)
//         {
//           for (int p = 0; p < num_solns; ++p)
//           {
//             int cur_inliers = 0;
//             for (int k = 0; k < num_correspondences; ++k)
//             {
//               Vector3d reproj_point = soln_projection[p] * points3D[k].homogeneous();
//               const double reproj_error = (reproj_point.hnormalized() - points2D[k]).norm();
//               if (reproj_error <= pixel_threshold)
//               {
//                 cur_inliers++;
//               }
//             }
//             //get the best solution
//             if (cur_inliers > max_inliers)
//             {
//               max_inliers = cur_inliers;
//               best_solution_id = p;
//             }
//           }
//           //for each RANSAC round, store the best solution.
//           if (max_inliers > 4)
//           {
//             all_ransac_solution.push_back(soln_projection[best_solution_id]);
//             all_inlier_nb.push_back(std::make_pair(max_inliers, all_inlier_nb_index));
//             all_inlier_nb_index++;
//           }
//         }
//       }

//       //std::cout << "there are " << all_ransac_solution.size() << " potential solutions" << std::endl;
//       if (all_ransac_solution.size() > 0)
//       {
//         // //sort based on inliers
//         std::sort(all_inlier_nb.begin(), all_inlier_nb.end(), sortasce);
//         //std::cout << "size of all inleirs nb " << all_inlier_nb.size() << std::endl;
//         std::vector<Matrix<double, 3, 4> > best_ransac_solutions;
//         best_ransac_solutions.clear();
//         float best_inlier = (float)all_inlier_nb[0].first;
//         int tolerate_threshold = (int)(best_inlier * 0.7);
//         int max_test_solution = 10;


//         for (int j = 0; j < all_inlier_nb.size(); j++)
//         {
//           if (j >= max_test_solution)
//             break;
//           //get the potentially best solutions.
//           if (all_inlier_nb[j].first >= tolerate_threshold)
//           {
//             std::cout << "matches " << num_correspondences << " inliers " << all_inlier_nb[j].first << std::endl;
//             best_ransac_solutions.push_back(all_ransac_solution[all_inlier_nb[j].second]);
//           }
//           else
//             break;
//         }

//         std::vector< std::pair< double, int > > focal_length_list;
//         std::vector< double > error_list;
//         error_list.clear();
//         focal_length_list.clear();
//         for (int j = 0; j < best_ransac_solutions.size(); j++)
//         {
//           //   get camera position
//           Matrix3d calibration_matrix_1;
//           Vector3d rotation_1, position_1;
//           theia::DecomposeProjectionMatrix(best_ransac_solutions[j],
//                                            &calibration_matrix_1,
//                                            &rotation_1,
//                                            &position_1);

//           // std::cout << "error "
//           //           << (camera_pos[cam_id] - position_1).norm()
//           //           << " focal " << calibration_matrix_1(0, 0) << std::endl;

//           focal_length_list.push_back(std::make_pair(calibration_matrix_1(0, 0), j));
//           error_list.push_back((camera_pos[cam_id] - position_1).norm());
//         }
//         int median_element = focal_length_list.size() / 2;
//         std::sort(focal_length_list.begin(), focal_length_list.end(), compare_score);
//         //std::cout << "median " << median_element << " length " << focal_length_list[median_element].first << std::endl;
//         std::cout << cam_id << " error " << error_list[focal_length_list[median_element].second] << " ";
//         best_camera_matrix = best_ransac_solutions[focal_length_list[median_element].second];
//         // std::cout << "camera center " << camera_pos[cam_id][0] << " " << camera_pos[cam_id][1]
//         // << " " << camera_pos[cam_id][2] << std::endl;


//         //use the best camera pose to re-evaluate all the matches
//         std::vector<Eigen::Vector2d> potential_points2D;
//         std::vector<Eigen::Vector3d> potential_points3D;
//         potential_points2D.clear();
//         potential_points3D.clear();
//         int potential_inliers = 0;
//         for (int i = 0; i < full_points2D.size(); i++ )
//         {
//           Vector3d full_reproj_point = best_camera_matrix * full_points3D[i].homogeneous();
//           const double full_reproj_error = (full_reproj_point.hnormalized() - full_points2D[i]).norm();
//           if (full_reproj_error <= 10)
//           {
//             potential_inliers++;
//             potential_points2D.push_back(full_points2D[i]);
//             potential_points3D.push_back(full_points3D[i]);
//           }
//         }
//         std::cout << "potential inliers " << potential_inliers << std::endl;

//         end_timer = clock();
//         std::cout << float( end_timer - begin_timer ) /  CLOCKS_PER_SEC << std::endl;
//         ofs_focal_length << cam_id << " " << focal_length_list[median_element].first << std::endl;
//         ofs_2d << cam_id << " " << potential_points2D.size() << std::endl;

//         for (int i = 0; i < potential_points2D.size(); i++)
//         {
//           ofs_2d << potential_points2D[i][0] << " " << potential_points2D[i][1] << std::endl;
//         }
//         ofs_3d << cam_id << " " << potential_points3D.size() << std::endl;
//         for (int i = 0; i < potential_points3D.size(); i++)
//         {
//           ofs_3d << potential_points3D[i][0] << " " << potential_points3D[i][1] << " " << potential_points3D[i][2] << std::endl;
//           // std::cout << potential_points3D[i][0] << " " << potential_points3D[i][1] << " " <<
//           // potential_points3D[i][2] << " 255 0 0" << std::endl;
//         }

//         //store the focal length and the correspondences






//         // //second round ransac

//         // int second_num_correspondences = 0;
//         // //check the consistency.
//         // if (potential_points3D.size() == potential_points2D.size())
//         // {
//         //   second_num_correspondences = potential_points3D.size();
//         // }
//         // //do the detailed ransac.
//         // Matrix<double, 3, 4> second_best_camera_matrix;
//         // if (second_num_correspondences > 0)
//         // {
//         //   int second_max_t = 1000;
//         //   std::vector<int> second_random_seqs;
//         //   second_random_seqs.resize(4 * second_max_t);
//         //   srand((int)time(0));
//         //   for (int i = 0; i < 4 * second_max_t; i++)
//         //   {
//         //     second_random_seqs[i] = rand() % second_num_correspondences;
//         //   }
//         //   std::vector<Vector2d> second_sample_points_2D(4);
//         //   std::vector<Vector3d> second_sample_points_3D(4);


//         //   int second_global_best_inliers = 0;

//         //   double second_pixel_threshold = 4.0f;
//         //   //int inlier_threshold = 11;
//         //   for (int  t = 0; t < second_max_t; t++)
//         //   {
//         //     second_sample_points_2D[0] = potential_points2D[second_random_seqs[4 * t]];
//         //     second_sample_points_3D[0] = potential_points3D[second_random_seqs[4 * t]];
//         //     second_sample_points_2D[1] = potential_points2D[second_random_seqs[4 * t + 1]];
//         //     second_sample_points_3D[1] = potential_points3D[second_random_seqs[4 * t + 1]];
//         //     second_sample_points_2D[2] = potential_points2D[second_random_seqs[4 * t + 2]];
//         //     second_sample_points_3D[2] = potential_points3D[second_random_seqs[4 * t + 2]];
//         //     second_sample_points_2D[3] = potential_points2D[second_random_seqs[4 * t + 3]];
//         //     second_sample_points_3D[3] = potential_points3D[second_random_seqs[4 * t + 3]];

//         //     //compute the pose using p4pf
//         //     std::vector<Matrix<double, 3, 4> > second_soln_projection;
//         //     int second_num_solns = theia::FourPointPoseAndFocalLength(
//         //                              second_sample_points_2D, second_sample_points_3D, &second_soln_projection);

//         //     int second_best_solution_id = 0;
//         //     int second_max_inliers = 0;
//         //     if (second_num_solns > 0)
//         //     {
//         //       for (int p = 0; p < second_num_solns; ++p)
//         //       {
//         //         int second_cur_inliers = 0;
//         //         for (int k = 0; k < second_num_correspondences; ++k)
//         //         {
//         //           Vector3d second_reproj_point = second_soln_projection[p] * potential_points3D[k].homogeneous();
//         //           const double second_reproj_error = (second_reproj_point.hnormalized() - potential_points2D[k]).norm();
//         //           if (second_reproj_error <= second_pixel_threshold)
//         //             second_cur_inliers++;
//         //         }
//         //         //get the best solution
//         //         if (second_cur_inliers > second_max_inliers)
//         //         {
//         //           second_max_inliers = second_cur_inliers;
//         //           second_best_solution_id = p;
//         //         }
//         //       }
//         //     }
//         //     //check the global best solution
//         //     if (second_max_inliers > second_global_best_inliers)
//         //     {
//         //       second_global_best_inliers = second_max_inliers;
//         //       second_best_camera_matrix = second_soln_projection[second_best_solution_id];
//         //     }
//         //   }

//         //   //get camera position
//         //   Matrix3d calibration_matrix;
//         //   Vector3d rotation, position;
//         //   theia::DecomposeProjectionMatrix(second_best_camera_matrix,
//         //                                    &calibration_matrix,
//         //                                    &rotation,
//         //                                    &position);



//         //   std::cout << "second " << cam_id << " error " << (camera_pos[cam_id] - position).norm() << std::endl;
//         //   //after ransac, if the best solution returns more than threshold inliers.
//         //   //return the best solution.
//         //   // std::cout << "all " << full_points2D.size()  << " potential inliers " << potential_inliers << std::endl
//         // }
//       }
//     }
//   }
//   in_2d_pos.close();
//   in_3d_pos.close();
//   ofs_2d.close();
//   ofs_3d.close();
//   ofs_focal_length.close();
//   return 0;
// }


















// #include <glog/logging.h>
// #include <gflags/gflags.h>
// #include <time.h>
// #include <theia/theia.h>
// #include <chrono>
// #include <string>
// #include <vector>
// #include <iostream>
// #include <fstream>
// using Eigen::Map;
// using Eigen::Matrix;
// using Eigen::Matrix3d;
// using Eigen::Vector3d;
// using Eigen::Vector2d;


// int main(int argc, char *argv[]) {

//   //the file of input 2D-3D correspondences.
//   std::ifstream in_2d_pos( argv[1], std::ios::in );
//   std::ifstream in_3d_pos( argv[2], std::ios::in );
//   std::ifstream camera_truth( argv[3], std::ios::in );
//   std::string pos_2d( argv[4] );
//   std::ofstream ofs_2d( pos_2d.c_str(), std::ios::out );
//   std::string pos_3d( argv[5] );
//   std::ofstream ofs_3d( pos_3d.c_str(), std::ios::out );
//   std::string focal_length_file( argv[6] );
//   std::ofstream ofs_focal_length( focal_length_file.c_str(), std::ios::out );

//   std::vector<Eigen::Vector2d> points2D;
//   std::vector<Eigen::Vector3d> points3D;
//   std::vector<Eigen::Vector2d> full_points2D;
//   std::vector<Eigen::Vector3d> full_points3D;

//   std::vector<Eigen::Vector3d> camera_pos;
//   camera_pos.clear();
//   while (!camera_truth.eof())
//   {
//     double cam_x; double cam_y; double cam_z;
//     camera_truth >> cam_x >> cam_y >> cam_z;
//     Eigen::Vector3d cam_3d(cam_x, cam_y, cam_z);
//     camera_pos.push_back(cam_3d);
//   }


//   while (!in_2d_pos.eof() && !in_3d_pos.eof())
//   {
//     int cam_id; int corrs_nb;
//     in_2d_pos >> cam_id >> corrs_nb;
//     points2D.clear();
//     points2D.reserve(corrs_nb);
//     for (int i = 0; i < corrs_nb; i++)
//     {
//       double x_2d; double y_2d;
//       in_2d_pos >> x_2d >> y_2d;
//       Eigen::Vector2d insert_2d(x_2d, -1 * y_2d);
//       //Eigen::Vector2d insert_2d(x_2d,  y_2d);
//       points2D.push_back(insert_2d);
//     }

//     //read the full set of correspondences
//     int full_cam_id; int full_corrs_nb;
//     in_2d_pos >> full_cam_id >> full_corrs_nb;
//     full_points2D.clear();
//     full_points2D.reserve(full_corrs_nb);
//     for (int i = 0; i < full_corrs_nb; i++)
//     {
//       double full_x_2d; double full_y_2d;
//       in_2d_pos >> full_x_2d >> full_y_2d;
//       Eigen::Vector2d full_insert_2d(full_x_2d, -1 * full_y_2d);
//       //Eigen::Vector2d insert_2d(x_2d,  y_2d);
//       full_points2D.push_back(full_insert_2d);
//     }

//     // std::cout << "corrs " << cam_id << " " << corrs_nb  <<
//     //           " full " << full_cam_id << " " << full_corrs_nb << std::endl;

//     int cam_id_3d; int corrs_nb_3d;
//     in_3d_pos >> cam_id_3d >> corrs_nb_3d;
//     points3D.clear();
//     points3D.reserve(corrs_nb_3d);
//     for (int i = 0; i < corrs_nb_3d; i++)
//     {
//       double x_3d; double y_3d; double z_3d;
//       in_3d_pos >> x_3d >> y_3d >> z_3d;
//       Eigen::Vector3d insert_3d( x_3d, y_3d, z_3d);
//       points3D.push_back(insert_3d);
//     }

//     //read the full set of correspondences
//     int full_cam_id_3d; int full_corrs_nb_3d;
//     in_3d_pos >> full_cam_id_3d >> full_corrs_nb_3d;
//     full_points3D.clear();
//     full_points3D.reserve(full_corrs_nb_3d);
//     for (int i = 0; i < full_corrs_nb_3d; i++)
//     {
//       double full_x_3d; double full_y_3d; double full_z_3d;
//       in_3d_pos >> full_x_3d >> full_y_3d >> full_z_3d;
//       Eigen::Vector3d full_insert_3d( full_x_3d, full_y_3d, full_z_3d);
//       full_points3D.push_back(full_insert_3d);
//     }
//     // std::cout << "corrs " << cam_id_3d << " " << corrs_nb_3d  <<
//     //           " full " << full_cam_id_3d << " " << full_corrs_nb_3d << std::endl;
//     int num_correspondences = 0;
//     //check the consistency.
//     if (points3D.size() == points2D.size())
//     {
//       num_correspondences = points3D.size();
//     }

//     //do the detailed ransac.
//     Matrix<double, 3, 4> best_camera_matrix;
//     Matrix3d calibration_matrix_1;
//     Vector3d rotation_1, position_1;
//     if (num_correspondences > 0)
//     {

//       // for (int p = 0; p < 1000; p++)
//       // {
//       int max_t = 1000;
//       std::vector<int> random_seqs;
//       random_seqs.resize(4 * max_t);
//       srand((int)time(0));
//       for (int i = 0; i < 4 * max_t; i++)
//       {
//         random_seqs[i] = rand() % num_correspondences;
//         // std::cout << random_seqs[i] << std::endl;
//       }
//       std::vector<Vector2d> sample_points_2D(4);
//       std::vector<Vector3d> sample_points_3D(4);


//       int global_best_inliers = 0;

//       double pixel_threshold = 10.0f;

//       std::vector<int> inleir_set;
//       std::vector<int> best_inleir_set;
//       std::vector<int> global_best_inleir_set;
//       best_inleir_set.clear();
//       global_best_inleir_set.clear();
//       //int inlier_threshold = 11;
//       for (int  t = 0; t < max_t; t++)
//       {
//         sample_points_2D[0] = points2D[random_seqs[4 * t]];
//         sample_points_3D[0] = points3D[random_seqs[4 * t]];
//         sample_points_2D[1] = points2D[random_seqs[4 * t + 1]];
//         sample_points_3D[1] = points3D[random_seqs[4 * t + 1]];
//         sample_points_2D[2] = points2D[random_seqs[4 * t + 2]];
//         sample_points_3D[2] = points3D[random_seqs[4 * t + 2]];
//         sample_points_2D[3] = points2D[random_seqs[4 * t + 3]];
//         sample_points_3D[3] = points3D[random_seqs[4 * t + 3]];

//         //compute the pose using p4pf
//         std::vector<Matrix<double, 3, 4> > soln_projection;
//         int num_solns = theia::FourPointPoseAndFocalLength(
//                           sample_points_2D, sample_points_3D, &soln_projection);

//         int best_solution_id = 0;
//         int max_inliers = 0;
//         if (num_solns > 0)
//         {
//           for (int p = 0; p < num_solns; ++p)
//           {
//             inleir_set.clear();
//             int cur_inliers = 0;
//             for (int k = 0; k < num_correspondences; ++k)
//             {
//               Vector3d reproj_point = soln_projection[p] * points3D[k].homogeneous();
//               if (reproj_point[2] > 0)
//               {
//                 const double reproj_error = (reproj_point.hnormalized() - points2D[k]).norm();
//                 if (reproj_error <= pixel_threshold)
//                 {
//                   cur_inliers++;
//                   inleir_set.push_back(k);
//                 }
//               }

//               // const double reproj_error = (reproj_point.hnormalized() - points2D[k]).norm();
//               // if (reproj_error <= pixel_threshold)
//               // {
//               //   cur_inliers++;
//               //   inleir_set.push_back(k);
//               // }
//             }
//             //get the best solution
//             if (cur_inliers > max_inliers)
//             {
//               max_inliers = cur_inliers;
//               best_solution_id = p;
//               best_inleir_set = inleir_set;
//             }
//           }
//         }
//         //check the global best solution
//         if (max_inliers > global_best_inliers)
//         {
//           global_best_inliers = max_inliers;
//           best_camera_matrix = soln_projection[best_solution_id];
//           global_best_inleir_set = best_inleir_set;
//         }
//       }

//       //get camera position

//       theia::DecomposeProjectionMatrix(best_camera_matrix,
//                                        &calibration_matrix_1,
//                                        &rotation_1,
//                                        &position_1);

//       std::cout << "first round " << cam_id << " " << num_correspondences << " " <<
//                 global_best_inliers << " " << (camera_pos[cam_id] - position_1).norm()
//                 << " focal length " << calibration_matrix_1(0, 0) << std::endl;;



//       //first_round_quality = (float)global_best_inliers / (float)num_correspondences;
//       //first_round_error = (camera_pos[cam_id] - position_1).norm();

//       //after ransac, if the best solution returns more than threshold inliers.
//       //return the best solution.


//     }


//     //use the best camera pose to re-evaluate all the matches
//     std::vector<Eigen::Vector2d> potential_points2D;
//     std::vector<Eigen::Vector3d> potential_points3D;
//     potential_points2D.clear();
//     potential_points3D.clear();
//     int potential_inliers = 0;
//     for (int i = 0; i < full_points2D.size(); i++ )
//     {
//       Vector3d full_reproj_point = best_camera_matrix * full_points3D[i].homogeneous();
//       if (full_reproj_point[2] > 0)
//       {
//         const double full_reproj_error = (full_reproj_point.hnormalized() - full_points2D[i]).norm();
//         if (full_reproj_error <= 10)
//         {
//           potential_inliers++;
//           potential_points2D.push_back(full_points2D[i]);
//           potential_points3D.push_back(full_points3D[i]);
//         }
//       }
//     }

//     ofs_focal_length << cam_id << " " << calibration_matrix_1(0, 0) << std::endl;
//     ofs_2d << cam_id << " " << potential_points2D.size() << std::endl;

//     for (int i = 0; i < potential_points2D.size(); i++)
//     {
//       ofs_2d << potential_points2D[i][0] << " " << potential_points2D[i][1] << std::endl;
//     }
//     ofs_3d << cam_id << " " << potential_points3D.size() << std::endl;
//     for (int i = 0; i < potential_points3D.size(); i++)
//     {
//       ofs_3d << potential_points3D[i][0] << " " << potential_points3D[i][1] << " " << potential_points3D[i][2] << std::endl;
//       // std::cout << potential_points3D[i][0] << " " << potential_points3D[i][1] << " " <<
//       // potential_points3D[i][2] << " 255 0 0" << std::endl;
//     }

//     //store the focal length and the correspondences

//   }
//   in_2d_pos.close();
//   in_3d_pos.close();
//   return 0;
// }



















// #include <glog/logging.h>
// #include <gflags/gflags.h>
// #include <time.h>
// #include <theia/theia.h>
// #include <chrono>
// #include <string>
// #include <vector>
// #include <iostream>
// #include <fstream>
// using Eigen::Map;
// using Eigen::Matrix;
// using Eigen::Matrix3d;
// using Eigen::Vector3d;
// using Eigen::Vector2d;


// int main(int argc, char *argv[]) {

//   //the file of input 2D-3D correspondences.
//   std::ifstream in_2d_pos( argv[1], std::ios::in );
//   std::ifstream in_3d_pos( argv[2], std::ios::in );

//   std::vector<Eigen::Vector2d> points2D;
//   std::vector<Eigen::Vector3d> points3D;


//   while (!in_2d_pos.eof() && !in_3d_pos.eof())
//   {
//     int cam_id; int corrs_nb;
//     in_2d_pos >> cam_id >> corrs_nb;
//     points2D.clear();
//     points2D.reserve(corrs_nb);
//     for (int i = 0; i < corrs_nb; i++)
//     {
//       double x_2d; double y_2d;
//       in_2d_pos >> x_2d >> y_2d;
//       Eigen::Vector2d insert_2d(x_2d, -1* y_2d);
//       //Eigen::Vector2d insert_2d(x_2d,  y_2d);
//       points2D.push_back(insert_2d);
//     }

//     int cam_id_3d; int corrs_nb_3d;
//     in_3d_pos >> cam_id_3d >> corrs_nb_3d;
//     points3D.clear();
//     points3D.reserve(corrs_nb_3d);
//     for (int i = 0; i < corrs_nb_3d; i++)
//     {
//       double x_3d; double y_3d; double z_3d;
//       in_3d_pos >> x_3d >> y_3d >> z_3d;
//       Eigen::Vector3d insert_3d( x_3d, y_3d, z_3d);
//       points3D.push_back(insert_3d);
//     }

//     int num_correspondences = 0;
//     //check the consistency.
//     if (points3D.size() == points2D.size())
//     {
//       num_correspondences = points3D.size();
//     }
//     //do the detailed ransac.
//     if (num_correspondences > 0)
//     {
//       int max_t = 1000;
//       std::vector<int> random_seqs;
//       random_seqs.resize(4 * max_t);
//       srand((int)time(0));
//       for (int i = 0; i < 4 * max_t; i++)
//       {
//         random_seqs[i] = rand() % num_correspondences;
//       }
//       std::vector<Vector2d> sample_points_2D(4);
//       std::vector<Vector3d> sample_points_3D(4);


//       int global_best_inliers = 0;
//       Matrix<double, 3, 4> best_camera_matrix;
//       double pixel_threshold = 6.0f;
//       //int inlier_threshold = 11;
//       for (int  t = 0; t < max_t; t++)
//       {
//         sample_points_2D[0] = points2D[random_seqs[4 * t]];
//         sample_points_3D[0] = points3D[random_seqs[4 * t]];
//         sample_points_2D[1] = points2D[random_seqs[4 * t + 1]];
//         sample_points_3D[1] = points3D[random_seqs[4 * t + 1]];
//         sample_points_2D[2] = points2D[random_seqs[4 * t + 2]];
//         sample_points_3D[2] = points3D[random_seqs[4 * t + 2]];
//         sample_points_2D[3] = points2D[random_seqs[4 * t + 3]];
//         sample_points_3D[3] = points3D[random_seqs[4 * t + 3]];

//         //compute the pose using p4pf
//         std::vector<Matrix<double, 3, 4> > soln_projection;
//         int num_solns = theia::FourPointPoseAndFocalLength(
//                           sample_points_2D, sample_points_3D, &soln_projection);

//         int best_solution_id = 0;
//         int max_inliers = 0;
//         if (num_solns > 0)
//         {
//           for (int p = 0; p < num_solns; ++p)
//           {
//             int cur_inliers = 0;
//             for (int k = 0; k < num_correspondences; ++k)
//             {
//               Vector3d reproj_point = soln_projection[p] * points3D[k].homogeneous();
//               const double reproj_error = (reproj_point.hnormalized() - points2D[k]).norm();
//               if (reproj_error <= pixel_threshold)
//                 cur_inliers++;
//             }
//             //get the best solution
//             if (cur_inliers > max_inliers)
//             {
//               max_inliers = cur_inliers;
//               best_solution_id = p;
//             }
//           }
//         }
//         //check the global best solution
//         if (max_inliers > global_best_inliers)
//         {
//           global_best_inliers = max_inliers;
//           best_camera_matrix = soln_projection[best_solution_id];
//         }
//       }

//       std::cout << cam_id << " " << num_correspondences << " " <<  global_best_inliers << std::endl;
//       //after ransac, if the best solution returns more than threshold inliers.
//       //return the best solution.
//     }

//   }
//   return 0;
// }
















// #include <glog/logging.h>
// #include <gflags/gflags.h>
// #include <time.h>
// #include <theia/theia.h>
// #include <chrono>
// #include <string>
// #include <vector>
// #include <iostream>
// #include <fstream>
// using Eigen::Map;
// using Eigen::Matrix;
// using Eigen::Matrix3d;
// using Eigen::Vector3d;
// using Eigen::Vector2d;


// int main(int argc, char *argv[]) {

//   //the file of input 2D-3D correspondences.
//   std::ifstream in_2d_pos( argv[1], std::ios::in );
//   std::ifstream in_3d_pos( argv[2], std::ios::in );

//   std::vector<Eigen::Vector2d> points2D;
//   std::vector<Eigen::Vector3d> points3D;


//   while (!in_2d_pos.eof() && !in_3d_pos.eof())
//   {
//     int cam_id; int corrs_nb;
//     in_2d_pos >> cam_id >> corrs_nb;
//     points2D.clear();
//     points2D.reserve(corrs_nb);
//     for (int i = 0; i < corrs_nb; i++)
//     {
//       double x_2d; double y_2d;
//       in_2d_pos >> x_2d >> y_2d;
//       Eigen::Vector2d insert_2d(x_2d, -1* y_2d);
//       //Eigen::Vector2d insert_2d(x_2d,  y_2d);
//       points2D.push_back(insert_2d);
//     }

//     int cam_id_3d; int corrs_nb_3d;
//     in_3d_pos >> cam_id_3d >> corrs_nb_3d;
//     points3D.clear();
//     points3D.reserve(corrs_nb_3d);
//     for (int i = 0; i < corrs_nb_3d; i++)
//     {
//       double x_3d; double y_3d; double z_3d;
//       in_3d_pos >> x_3d >> y_3d >> z_3d;
//       Eigen::Vector3d insert_3d( x_3d, y_3d, z_3d);
//       points3D.push_back(insert_3d);
//     }

//     int num_correspondences = 0;
//     //check the consistency.
//     if (points3D.size() == points2D.size())
//     {
//       num_correspondences = points3D.size();
//     }
//     //do the detailed ransac.
//     if (num_correspondences > 0)
//     {
//       int max_t = 1000;
//       std::vector<int> random_seqs;
//       random_seqs.resize(4 * max_t);
//       srand((int)time(0));
//       for (int i = 0; i < 4 * max_t; i++)
//       {
//         random_seqs[i] = rand() % num_correspondences;
//       }
//       std::vector<Vector2d> sample_points_2D(4);
//       std::vector<Vector3d> sample_points_3D(4);


//       int global_best_inliers = 0;
//       Matrix<double, 3, 4> best_camera_matrix;
//       double pixel_threshold = 6.0f;
//       //int inlier_threshold = 11;
//       for (int  t = 0; t < max_t; t++)
//       {
//         sample_points_2D[0] = points2D[random_seqs[4 * t]];
//         sample_points_3D[0] = points3D[random_seqs[4 * t]];
//         sample_points_2D[1] = points2D[random_seqs[4 * t + 1]];
//         sample_points_3D[1] = points3D[random_seqs[4 * t + 1]];
//         sample_points_2D[2] = points2D[random_seqs[4 * t + 2]];
//         sample_points_3D[2] = points3D[random_seqs[4 * t + 2]];
//         sample_points_2D[3] = points2D[random_seqs[4 * t + 3]];
//         sample_points_3D[3] = points3D[random_seqs[4 * t + 3]];

//         //compute the pose using p4pf
//         std::vector<Matrix<double, 3, 4> > soln_projection;
//         int num_solns = theia::FourPointPoseAndFocalLength(
//                           sample_points_2D, sample_points_3D, &soln_projection);

//         int best_solution_id = 0;
//         int max_inliers = 0;
//         if (num_solns > 0)
//         {
//           for (int p = 0; p < num_solns; ++p)
//           {
//             int cur_inliers = 0;
//             for (int k = 0; k < num_correspondences; ++k)
//             {
//               Vector3d reproj_point = soln_projection[p] * points3D[k].homogeneous();
//               const double reproj_error = (reproj_point.hnormalized() - points2D[k]).norm();
//               if (reproj_error <= pixel_threshold)
//                 cur_inliers++;
//             }
//             //get the best solution
//             if (cur_inliers > max_inliers)
//             {
//               max_inliers = cur_inliers;
//               best_solution_id = p;
//             }
//           }
//         }
//         //check the global best solution
//         if (max_inliers > global_best_inliers)
//         {
//           global_best_inliers = max_inliers;
//           best_camera_matrix = soln_projection[best_solution_id];
//         }
//       }

//       std::cout << cam_id << " " << num_correspondences << " " <<  global_best_inliers << std::endl;
//       //after ransac, if the best solution returns more than threshold inliers.
//       //return the best solution.
//     }

//   }
//   return 0;
// }





























#include <glog/logging.h>
#include <gflags/gflags.h>
#include <time.h>
#include <theia/theia.h>
#include <chrono>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector2d;


int main(int argc, char *argv[]) {

  //the file of input 2D-3D correspondences.
  std::ifstream in_2d_pos( argv[1], std::ios::in );
  std::ifstream in_3d_pos( argv[2], std::ios::in );
  std::ifstream camera_truth( argv[3], std::ios::in );

  std::vector<Eigen::Vector2d> points2D;
  std::vector<Eigen::Vector3d> points3D;
  std::vector<Eigen::Vector2d> full_points2D;
  std::vector<Eigen::Vector3d> full_points3D;

  std::vector<Eigen::Vector3d> camera_pos;
  camera_pos.clear();
  while (!camera_truth.eof())
  {
    double cam_x; double cam_y; double cam_z;
    camera_truth >> cam_x >> cam_y >> cam_z;
    Eigen::Vector3d cam_3d(cam_x, cam_y, cam_z);
    camera_pos.push_back(cam_3d);
  }


  while (!in_2d_pos.eof() && !in_3d_pos.eof())
  {
    int cam_id; int corrs_nb;
    in_2d_pos >> cam_id >> corrs_nb;
    points2D.clear();
    points2D.reserve(corrs_nb);
    for (int i = 0; i < corrs_nb; i++)
    {
      double x_2d; double y_2d;
      in_2d_pos >> x_2d >> y_2d;
      Eigen::Vector2d insert_2d(x_2d, -1 * y_2d);
      //Eigen::Vector2d insert_2d(x_2d,  y_2d);
      points2D.push_back(insert_2d);
    }

    //read the full set of correspondences
    int full_cam_id; int full_corrs_nb;
    in_2d_pos >> full_cam_id >> full_corrs_nb;
    full_points2D.clear();
    full_points2D.reserve(full_corrs_nb);
    for (int i = 0; i < full_corrs_nb; i++)
    {
      double full_x_2d; double full_y_2d;
      in_2d_pos >> full_x_2d >> full_y_2d;
      Eigen::Vector2d full_insert_2d(full_x_2d, -1 * full_y_2d);
      //Eigen::Vector2d insert_2d(x_2d,  y_2d);
      full_points2D.push_back(full_insert_2d);
    }

    // std::cout << "corrs " << cam_id << " " << corrs_nb  <<
    //           " full " << full_cam_id << " " << full_corrs_nb << std::endl;

    int cam_id_3d; int corrs_nb_3d;
    in_3d_pos >> cam_id_3d >> corrs_nb_3d;
    points3D.clear();
    points3D.reserve(corrs_nb_3d);
    for (int i = 0; i < corrs_nb_3d; i++)
    {
      double x_3d; double y_3d; double z_3d;
      in_3d_pos >> x_3d >> y_3d >> z_3d;
      Eigen::Vector3d insert_3d( x_3d, y_3d, z_3d);
      points3D.push_back(insert_3d);
    }

    //read the full set of correspondences
    int full_cam_id_3d; int full_corrs_nb_3d;
    in_3d_pos >> full_cam_id_3d >> full_corrs_nb_3d;
    full_points3D.clear();
    full_points3D.reserve(full_corrs_nb_3d);
    for (int i = 0; i < full_corrs_nb_3d; i++)
    {
      double full_x_3d; double full_y_3d; double full_z_3d;
      in_3d_pos >> full_x_3d >> full_y_3d >> full_z_3d;
      Eigen::Vector3d full_insert_3d( full_x_3d, full_y_3d, full_z_3d);
      full_points3D.push_back(full_insert_3d);
    }
    // std::cout << "corrs " << cam_id_3d << " " << corrs_nb_3d  <<
    //           " full " << full_cam_id_3d << " " << full_corrs_nb_3d << std::endl;
    int num_correspondences = 0;
    //check the consistency.
    if (points3D.size() == points2D.size())
    {
      num_correspondences = points3D.size();
    }

    for (int k = 0; k < num_correspondences; ++k)
    {

      // double color_r = ((double) rand() / (RAND_MAX));
      // double color_g = ((double) rand() / (RAND_MAX));
      // double color_b = ((double) rand() / (RAND_MAX));

      std::cout << "plot(" << points2D[k][0] + 800.0 << "," <<
                535.0 + points2D[k][1] << ",'.','color',[" << 1 << "," << 1 << "," << 0 << "],'MarkerSize',15)" << std::endl;
    }

    //float first_round_quality = 0;
    // float second_round_quality = 0;
    // float first_round_error = 0;
    // float second_round_error = 0;

    //do the detailed ransac.
    Matrix<double, 3, 4> best_camera_matrix;
    if (num_correspondences > 0)
    {

      // for (int p = 0; p < 1000; p++)
      // {
        int max_t = 1000;
        std::vector<int> random_seqs;
        random_seqs.resize(4 * max_t);
        srand((int)time(0));
        for (int i = 0; i < 4 * max_t; i++)
        {
          random_seqs[i] = rand() % num_correspondences;
          // std::cout << random_seqs[i] << std::endl;
        }
        std::vector<Vector2d> sample_points_2D(4);
        std::vector<Vector3d> sample_points_3D(4);


        int global_best_inliers = 0;

        double pixel_threshold = 8.0f;

        std::vector<int> inleir_set;
        std::vector<int> best_inleir_set;
        std::vector<int> global_best_inleir_set;
        best_inleir_set.clear();
        global_best_inleir_set.clear();
        //int inlier_threshold = 11;
        for (int  t = 0; t < max_t; t++)
        {
          sample_points_2D[0] = points2D[random_seqs[4 * t]];
          sample_points_3D[0] = points3D[random_seqs[4 * t]];
          sample_points_2D[1] = points2D[random_seqs[4 * t + 1]];
          sample_points_3D[1] = points3D[random_seqs[4 * t + 1]];
          sample_points_2D[2] = points2D[random_seqs[4 * t + 2]];
          sample_points_3D[2] = points3D[random_seqs[4 * t + 2]];
          sample_points_2D[3] = points2D[random_seqs[4 * t + 3]];
          sample_points_3D[3] = points3D[random_seqs[4 * t + 3]];

          //compute the pose using p4pf
          std::vector<Matrix<double, 3, 4> > soln_projection;
          int num_solns = theia::FourPointPoseAndFocalLength(
                            sample_points_2D, sample_points_3D, &soln_projection);

          int best_solution_id = 0;
          int max_inliers = 0;
          if (num_solns > 0)
          {
            for (int p = 0; p < num_solns; ++p)
            {
              inleir_set.clear();
              int cur_inliers = 0;
              for (int k = 0; k < num_correspondences; ++k)
              {
                Vector3d reproj_point = soln_projection[p] * points3D[k].homogeneous();
                if (reproj_point[2] > 0)
                {
                  const double reproj_error = (reproj_point.hnormalized() - points2D[k]).norm();
                  if (reproj_error <= pixel_threshold)
                  {
                    cur_inliers++;
                    inleir_set.push_back(k);
                  }
                }

                // const double reproj_error = (reproj_point.hnormalized() - points2D[k]).norm();
                // if (reproj_error <= pixel_threshold)
                // {
                //   cur_inliers++;
                //   inleir_set.push_back(k);
                // }
              }
              //get the best solution
              if (cur_inliers > max_inliers)
              {
                max_inliers = cur_inliers;
                best_solution_id = p;
                best_inleir_set = inleir_set;
              }
            }
          }
          //check the global best solution
          if (max_inliers > global_best_inliers)
          {
            global_best_inliers = max_inliers;
            best_camera_matrix = soln_projection[best_solution_id];
            global_best_inleir_set = best_inleir_set;
          }
        }

        //get camera position
        Matrix3d calibration_matrix_1;
        Vector3d rotation_1, position_1;
        theia::DecomposeProjectionMatrix(best_camera_matrix,
                                         &calibration_matrix_1,
                                         &rotation_1,
                                         &position_1);

        std::cout << "first round " << cam_id << " " << num_correspondences << " " <<
                  global_best_inliers << " " << (camera_pos[cam_id] - position_1).norm()
                  << " focal length " << calibration_matrix_1(0, 0) << std::endl;;



        //first_round_quality = (float)global_best_inliers / (float)num_correspondences;
        //first_round_error = (camera_pos[cam_id] - position_1).norm();

        //after ransac, if the best solution returns more than threshold inliers.
        //return the best solution.


        for (int j = 0; j < global_best_inleir_set.size(); j++)
        {
          // double color_r = ((double) rand() / (RAND_MAX));
          // double color_g = ((double) rand() / (RAND_MAX));
          // double color_b = ((double) rand() / (RAND_MAX));


         // std::cout << points2D[global_best_inleir_set[j]][0] << " " << points2D[global_best_inleir_set[j]][1] << std::endl;


          std::cout << "plot(" << points2D[global_best_inleir_set[j]][0] + 800.0 << "," <<
                    535.0 + points2D[global_best_inleir_set[j]][1] << ",'.','color',[" << 0 << "," << 1 << "," << 0 << "],'MarkerSize',15)" << std::endl;
          // std::cout << "plot(" << points2D[global_best_inleir_set[j]][0] << "," <<
          //           -1.0 * points2D[global_best_inleir_set[j]][1] << ",'.','color',[" << color_r << "," << color_g << "," << color_b << "],'MarkerSize',15)" << std::endl;

        }
      // }



    }


    // //use the best camera pose to re-evaluate all the matches
    // std::vector<Eigen::Vector2d> potential_points2D;
    // std::vector<Eigen::Vector3d> potential_points3D;
    // potential_points2D.clear();
    // potential_points3D.clear();
    // int potential_inliers = 0;
    // for (int i = 0; i < full_points2D.size(); i++ )
    // {
    //   Vector3d full_reproj_point = best_camera_matrix * full_points3D[i].homogeneous();
    //   if (full_reproj_point[2] > 0)
    //   {
    //     const double full_reproj_error = (full_reproj_point.hnormalized() - full_points2D[i]).norm();
    //     if (full_reproj_error <= 6)
    //     {
    //       potential_inliers++;
    //       potential_points2D.push_back(full_points2D[i]);
    //       potential_points3D.push_back(full_points3D[i]);
    //     }
    //   }
    // }
    // // std::cout << "all " << full_points2D.size()  << " potential inliers " << potential_inliers << std::endl;


    // int second_num_correspondences = 0;
    // //check the consistency.
    // if (potential_points3D.size() == potential_points2D.size())
    // {
    //   second_num_correspondences = potential_points3D.size();
    // }
    // //do the detailed ransac.
    // Matrix<double, 3, 4> second_best_camera_matrix;

    // // for(int i = 0;i < potential_points3D.size(); i++)
    // // {
    // //   std::cout << potential_points3D[i][0] << " "
    // //   << potential_points3D[i][1] << " " <<
    // //   potential_points3D[i][2] << std::endl;
    // // }
    // if (second_num_correspondences > 0)
    // {
    //   int second_max_t = 1000;
    //   std::vector<int> second_random_seqs;
    //   second_random_seqs.resize(4 * second_max_t);
    //   srand((int)time(0));
    //   for (int i = 0; i < 4 * second_max_t; i++)
    //   {
    //     second_random_seqs[i] = rand() % second_num_correspondences;
    //   }
    //   std::vector<Vector2d> second_sample_points_2D(4);
    //   std::vector<Vector3d> second_sample_points_3D(4);


    //   int second_global_best_inliers = 0;

    //   double second_pixel_threshold = 4.0f;

    //   std::vector<int> second_inleir_set;
    //   std::vector<int> second_best_inleir_set;
    //   std::vector<int> second_global_best_inleir_set;
    //   second_best_inleir_set.clear();
    //   second_global_best_inleir_set.clear();



    //   //int inlier_threshold = 11;
    //   for (int  t = 0; t < second_max_t; t++)
    //   {
    //     second_sample_points_2D[0] = potential_points2D[second_random_seqs[4 * t]];
    //     second_sample_points_3D[0] = potential_points3D[second_random_seqs[4 * t]];
    //     second_sample_points_2D[1] = potential_points2D[second_random_seqs[4 * t + 1]];
    //     second_sample_points_3D[1] = potential_points3D[second_random_seqs[4 * t + 1]];
    //     second_sample_points_2D[2] = potential_points2D[second_random_seqs[4 * t + 2]];
    //     second_sample_points_3D[2] = potential_points3D[second_random_seqs[4 * t + 2]];
    //     second_sample_points_2D[3] = potential_points2D[second_random_seqs[4 * t + 3]];
    //     second_sample_points_3D[3] = potential_points3D[second_random_seqs[4 * t + 3]];

    //     //compute the pose using p4pf
    //     std::vector<Matrix<double, 3, 4> > second_soln_projection;
    //     int second_num_solns = theia::FourPointPoseAndFocalLength(
    //                              second_sample_points_2D, second_sample_points_3D, &second_soln_projection);

    //     int second_best_solution_id = 0;
    //     int second_max_inliers = 0;
    //     if (second_num_solns > 0)
    //     {
    //       for (int p = 0; p < second_num_solns; ++p)
    //       {
    //         second_inleir_set.clear();
    //         int second_cur_inliers = 0;
    //         for (int k = 0; k < second_num_correspondences; ++k)
    //         {
    //           Vector3d second_reproj_point = second_soln_projection[p] * potential_points3D[k].homogeneous();
    //           if (second_reproj_point[2] > 0)
    //           {
    //             const double second_reproj_error = (second_reproj_point.hnormalized() - potential_points2D[k]).norm();
    //             if (second_reproj_error <= second_pixel_threshold)
    //             {

    //               second_cur_inliers++;
    //               second_inleir_set.push_back(k);
    //             }
    //           }
    //         }
    //         //get the best solution
    //         if (second_cur_inliers > second_max_inliers)
    //         {
    //           second_max_inliers = second_cur_inliers;
    //           second_best_solution_id = p;
    //           second_best_inleir_set = second_inleir_set;
    //         }
    //       }
    //     }
    //     //check the global best solution
    //     if (second_max_inliers > second_global_best_inliers)
    //     {
    //       second_global_best_inliers = second_max_inliers;
    //       second_best_camera_matrix = second_soln_projection[second_best_solution_id];
    //       second_global_best_inleir_set = second_best_inleir_set;
    //     }
    //   }

    //   //get camera position
    //   Matrix3d calibration_matrix;
    //   Vector3d rotation, position;
    //   theia::DecomposeProjectionMatrix(second_best_camera_matrix,
    //                                    &calibration_matrix,
    //                                    &rotation,
    //                                    &position);



    //   std::cout << "second round " << cam_id << " " << second_num_correspondences << " " <<  second_global_best_inliers
    //             << " " << (camera_pos[cam_id] - position).norm() <<
    //             " focal length " << calibration_matrix(0, 0) << std::endl;

    //   // std::cout << "camera position " << position << std::endl;
    //   // for (int j = 0; j < second_global_best_inleir_set.size(); j++)
    //   // {

    //   //   std::cout << potential_points3D[second_global_best_inleir_set[j]][0] << " " <<
    //   //             potential_points3D[second_global_best_inleir_set[j]][1] << " " <<
    //   //             potential_points3D[second_global_best_inleir_set[j]][2] << std::endl;

    //   // }



    //   // second_round_quality = (float)second_global_best_inliers / (float)second_num_correspondences;
    //   // second_round_error = (camera_pos[cam_id] - position).norm();

    //   // if(second_round_quality > 0.75)
    //   //   std::cout << second_round_error << std::endl;
    //   // else
    //   //   std::cout << first_round_error << std::endl;


    //   //after ransac, if the best solution returns more than threshold inliers.
    //   //return the best solution.

    //   // for (int j = 0; j < second_global_best_inleir_set.size(); j++)
    //   // {
    //   //   double color_r = ((double) rand() / (RAND_MAX));
    //   //   double color_g = ((double) rand() / (RAND_MAX));
    //   //   double color_b = ((double) rand() / (RAND_MAX));


    //   //  // std::cout << points2D[second_global_best_inleir_set[j]][0] << " " << points2D[second_global_best_inleir_set[j]][1] << std::endl;

    //   //   std::cout << "plot(" << points2D[second_global_best_inleir_set[j]][0] + 800.0 << "," <<
    //   //             535.0 + points2D[second_global_best_inleir_set[j]][1] << ",'.','color',[" << color_r << "," << color_g << "," << color_b << "],'MarkerSize',15)" << std::endl;
    //   //   // std::cout << "plot(" << points2D[global_best_inleir_set[j]][0] << "," <<
    //   //   //           -1.0 * points2D[global_best_inleir_set[j]][1] << ",'.','color',[" << color_r << "," << color_g << "," << color_b << "],'MarkerSize',15)" << std::endl;
    //   // }
    // }

  }
  in_2d_pos.close();
  in_3d_pos.close();
  return 0;
}
