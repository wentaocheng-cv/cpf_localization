Cascade_Parallel_Filtering 
wcheng005@e.ntu.edu.sg

------------
Requirements
------------
Cascade_Parallel_Filtering  uses the following libraries. If not specified otherwise, you have
to install them by yourself.
* FLANN.
   The Fast Library for Approximate Nearest Neighbors written Marius Muja and
   David G. Lowe, available at
   http://people.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN . ACG Localizer has
   been tested with version 1.6.11. Due to an API breaking change in version
   1.7.0, we suggest that you use 1.6.11, which (as of 02/29/2012) can still be
   found at http://people.cs.ubc.ca/~mariusm/uploads/FLANN/flann-1.6.11-src.zip
   We plan to update our source code to be compliant with newer versions of
   FLANN.   
* jhead
   A tool to read exif tags from jpg files, available at
   http://www.sentex.net/~mwandel/jhead/index.html . It is already included in
   this release of ACG Localizer.
* CMake (http://www.cmake.org/)
* Eigen 3.2.0 or newer (http://eigen.tuxfamily.org/)


------------
Installation
------------

Assuming that you unpacked Cascade_Parallel_Filtering into the directory
SOME_DIRECTORY/Cascade_Parallel_Filtering, you first have to edit the CMake files that are
responsible for finding the required libraries. These files are located in
SOME_DIRECTORY/Cascade_Parallel_Filtering/cmake. You have to edit the FindFLANN.cmake files to fill in the directories 
where the include and library
files of both libraries are located. 
cmake:
* cd SOME_DIRECTORY/Cascade_Parallel_Filtering
* mkdir build
* cd build
* cmake ..
If no library is missing and if CMake threw no errors you can build the software
by simply typing
* make

To get the best performance, make sure that you set the build type to "Release".

Typing 'make install' will install the generated executables in
SOME_DIRECTORY/Cascade_Parallel_Filtering/build/bin . Notice that to achieve timing results
similar to those in the paper, you have to
set the build type of CMake to "Release". You can do this as follows (assuming
that you have installed the gui-version of cmake):
* cd SOME_DIRECTORY/Cascade_Parallel_Filtering/build
* cmake-gui . (or "ccmake ." when you want to use the command line version of
cmake)
In the field "CMAKE_BUILD_TYPE" type "Release" (without quotation marks), then
click "Configure" and "Generate". Afterwards, compile the software with make.


-----
Usage
-----
In this code, we use the .info format to represent the 3D SfM models. 

For the meanings of parameters, please refer to the descriptions in codes.

1: First, you need to use the compute_desc_assignments to quantize feature descriptors in a SfM model into a visual vocabulary (either in integer mean per visual word or all descriptors)

./compute_desc_assignments aachen_cvpr2018_db.info 1 10000 aachen_cvpr2018_10k.txt aachen_10k_cvpr2018_branch100_mean.bin 6 1 1 100 1 

2: After quantization, you need to transfer the SIFT/RootSIFT descriptors into binary signatures. Following is the example:

./compute_hamming_threshold 10000 cluster/aachen_cvpr2018_10k.txt assignments/aachen_10k_cvpr2018_branch100_mean.bin projection_matrix/hamming_projection_matrix_64x128.txt hamming_thresholds/aachen_10k_cvpr2018_branch100_mean_hamming_threshold.txt

3: Then you can use our cascaded_parallel_filtering as following

./cascaded_parallel_filtering_aachenDayNight query_list/day_time_queries_with_intrinsics.txt 10000 cluster/aachen_cvpr2018_10k.txt hamming_thresholds/aachen_10k_cvpr2018_branch100_mean_hamming_threshold.txt 100 /home/desktop/Documents/Data/aachen_cvpr2018_db.info 16 3 20 50 0.3 0.8 output/aachen_cvpr_10k_2d_day.txt output/aachen_cvpr_10k_3d_day.txt 

./cascaded_parallel_filtering_aachenDayNight query_list/night_time_queries_with_intrinsics.txt 10000 cluster/aachen_cvpr2018_10k.txt hamming_thresholds/aachen_10k_cvpr2018_branch100_mean_hamming_threshold.txt 100 /home/desktop/Documents/Data/aachen_cvpr2018_db.info 16 3 20 50 0.3 0.8 output/aachen_cvpr_10k_2d_night.txt output/aachen_cvpr_10k_3d_night.txt 

4: The above program will generate two output files. One file stores the 2D positions of matches (first the matches for computing the auxiliary camera pose, second serve as visibility-wise match pool). In general, you have the following two options

a: suppose the focal length is unknown, you should use a p4p solver to compute the auxiliary camera pose, and filter the visibility-wise match pool with 10 pixel re-projection error threshold. After you obtain the final matches, you only need to run a p3p solver with voted focal length value for efficiently obtaining the final camera pose.

b: suppose camera calibration is fully known, you can simply apply a p3p solver to obtain the final camera pose. The following is an example:

./p3p_aachen aachen_iccv/aachen_cvpr_10k_2d_day.txt aachen_iccv/aachen_cvpr_10k_3d_day.txt aachen_iccv/aachen_cvpr_10k_pose_day.txt aachen_iccv/aachen_cvpr_10k_inlier_day.txt

./p3p_aachen aachen_iccv/aachen_cvpr_10k_2d_night.txt aachen_iccv/aachen_cvpr_10k_3d_night.txt aachen_iccv/aachen_cvpr_10k_pose_night.txt aachen_iccv/aachen_cvpr_10k_inlier_night.txt
