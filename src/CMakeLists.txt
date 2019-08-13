cmake_minimum_required (VERSION 2.6)

if (EXISTS "${CMAKE_SOURCE_DIR}/cmake")
  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
endif ()

#find_package (LAPACK)
find_package (FLANN)
find_package (Eigen)

# source and header of the exif reader
set (exif_SRC exif_reader/exif_reader.cc exif_reader/jhead-2.90/exif.cc exif_reader/jhead-2.90/gpsinfo.cc exif_reader/jhead-2.90/iptc.cc exif_reader/jhead-2.90/jhead.cc exif_reader/jhead-2.90/jpgfile.cc exif_reader/jhead-2.90/makernote.cc exif_reader/jhead-2.90/paths.cc )
set (exif_HDR exif_reader/exif_reader.hh exif_reader/jhead-2.90/jhead.hh)

# source and header of the feature library
set (features_SRC features/SIFT_loader.cc features/visual_words_handler.cc)
set (features_HDR features/SIFT_keypoint.hh features/SIFT_loader.hh features/visual_words_handler.hh)

# source and header of the math library
#set (math_SRC math/math.cc math/matrix3x3.cc math/matrix4x4.cc math/matrixbase.cc math/projmatrix.cc math/pseudorandomnrgen.cc math/SFMT_src/SFMT.cc )
#set (math_HDR math/math.hh math/matrix3x3.hh math/matrix4x4.hh math/matrixbase.hh math/projmatrix.hh  math/pseudorandomnrgen.hh math/SFMT_src/SFMT.hh math/SFMT_src/SFMT-params.hh math/SFMT_src/SFMT-params607.hh math/SFMT_src/SFMT-params1279.hh math/SFMT_src/SFMT-params2281.hh math/SFMT_src/SFMT-params4253.hh math/SFMT_src/SFMT-params11213.hh math/SFMT_src/SFMT-params19937.hh math/SFMT_src/SFMT-params44497.hh math/SFMT_src/SFMT-params86243.hh math/SFMT_src/SFMT-params132049.hh math/SFMT_src/SFMT-params216091.hh )

# source and header for the sfm functionality
set (sfm_SRC sfm/parse_bundler.cc sfm/bundler_camera.cc)
set (sfm_HDR sfm/parse_bundler.hh sfm/bundler_camera.hh)

# source and header for the 6-point pose solver
#set (solver_SRC solver/solverbase.cc solver/solverproj.cc)
#set (solver_HDR solver/solverbase.hh solver/solverproj.hh)

include_directories (
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN_INCLUDE_DIR}
  ${FLANN_INCLUDE_DIR}
)

link_directories (
  ${FLANN_LIBRARY_DIR}
  ${EIGEN_LIBRARY_DIR}
)



# set sources for the executables
#add_executable (Bundle2Info features/SIFT_loader.cc features/SIFT_keypoint.hh features/SIFT_loader.hh ${sfm_SRC} ${sfm_HDR} ${exif_SRC} ${exif_HDR} Bundle2Info )
add_executable (compute_desc_assignments compute_desc_assignments.cc ${sfm_SRC} ${sfm_HDR} ${features_SRC} ${exif_SRC} ${exif_HDR}  ${features_HDR} )
add_executable (cascaded_parallel_filtering ${exif_SRC} ${exif_HDR} ${features_SRC} ${features_HDR} timer.cc timer.hh ${sfm_SRC} ${sfm_HDR}  cascaded_parallel_filtering.cc )
#add_executable (acg_he_robot ${exif_SRC} ${exif_HDR} ${features_SRC} ${features_HDR} timer.cc timer.hh ${sfm_SRC} ${sfm_HDR}   acg_he_robot.cc )
#add_executable (acg_he_sf ${exif_SRC} ${exif_HDR} ${features_SRC} ${features_HDR} timer.cc timer.hh ${sfm_SRC} ${sfm_HDR}    acg_he_sf.cc )
add_executable (compute_hamming_threshold ${exif_SRC} ${exif_HDR} ${features_SRC} ${features_HDR} timer.cc timer.hh ${sfm_SRC} ${sfm_HDR}   compute_hamming_threshold.cc )
#add_executable (acg_he_sf_iccv ${exif_SRC} ${exif_HDR} ${features_SRC} ${features_HDR} timer.cc timer.hh ${sfm_SRC} ${sfm_HDR}    acg_he_sf_iccv.cc )
#add_executable (he_sf_root_sift ${exif_SRC} ${exif_HDR} ${features_SRC} ${features_HDR} timer.cc timer.hh ${sfm_SRC} ${sfm_HDR}    he_sf_root_sift.cc )
#add_executable (compute_hamming_threshold_128 ${exif_SRC} ${exif_HDR} ${features_SRC} ${features_HDR} timer.cc timer.hh ${sfm_SRC} ${sfm_HDR}    compute_hamming_threshold_128.cc )
#add_executable (he_sf_root_sift_128 ${exif_SRC} ${exif_HDR} ${features_SRC} ${features_HDR} timer.cc timer.hh ${sfm_SRC} ${sfm_HDR}    he_sf_root_sift_128.cc )
add_executable (cascaded_parallel_filtering_aachenDayNight ${exif_SRC} ${exif_HDR} ${features_SRC} ${features_HDR} timer.cc timer.hh ${sfm_SRC} ${sfm_HDR} cascaded_parallel_filtering_aachenDayNight.cc )

# set libraries to link against

#target_link_libraries (Bundle2Info
#)

target_link_libraries (compute_desc_assignments
  ${FLANN_LIBRARY}
)

target_link_libraries (cascaded_parallel_filtering
  ${EIGEN_LIBRARY}
  ${FLANN_LIBRARY}
)

target_link_libraries (cascaded_parallel_filtering_aachenDayNight
  ${EIGEN_LIBRARY}
  ${FLANN_LIBRARY}
)


#target_link_libraries (acg_he_robot
#  ${EIGEN_LIBRARY}
#  ${FLANN_LIBRARY}
#)

#target_link_libraries (acg_he_sf
 # ${EIGEN_LIBRARY}
 # ${FLANN_LIBRARY}
#)

#target_link_libraries (he_sf_root_sift
 # ${EIGEN_LIBRARY}
 # ${FLANN_LIBRARY}
#)

#target_link_libraries (he_sf_root_sift_128
 # ${EIGEN_LIBRARY}
 # ${FLANN_LIBRARY}
#)

#target_link_libraries (acg_he_sf_iccv
 # ${EIGEN_LIBRARY}
 # ${FLANN_LIBRARY}
#)

target_link_libraries (compute_hamming_threshold
  ${EIGEN_LIBRARY}
  ${FLANN_LIBRARY}
)

#target_link_libraries (compute_hamming_threshold_128
#  ${EIGEN_LIBRARY}
#  ${FLANN_LIBRARY}
#)


# install the executables

#install( PROGRAMS ${CMAKE_BINARY_DIR}/src/Bundle2Info
 #        DESTINATION ${CMAKE_BINARY_DIR}/bin)

install( PROGRAMS ${CMAKE_BINARY_DIR}/src/compute_desc_assignments
         DESTINATION ${CMAKE_BINARY_DIR}/bin) 

install( PROGRAMS ${CMAKE_BINARY_DIR}/src/cascaded_parallel_filtering
         DESTINATION ${CMAKE_BINARY_DIR}/bin) 

#install( PROGRAMS ${CMAKE_BINARY_DIR}/src/acg_he_robot
  #       DESTINATION ${CMAKE_BINARY_DIR}/bin) 

#install( PROGRAMS ${CMAKE_BINARY_DIR}/src/acg_he_sf
 #        DESTINATION ${CMAKE_BINARY_DIR}/bin) 

#install( PROGRAMS ${CMAKE_BINARY_DIR}/src/acg_he_sf_iccv
 #        DESTINATION ${CMAKE_BINARY_DIR}/bin) 

install( PROGRAMS ${CMAKE_BINARY_DIR}/src/cascaded_parallel_filtering_aachenDayNight
         DESTINATION ${CMAKE_BINARY_DIR}/bin)

install( PROGRAMS ${CMAKE_BINARY_DIR}/src/compute_hamming_threshold
         DESTINATION ${CMAKE_BINARY_DIR}/bin) 

#install( PROGRAMS ${CMAKE_BINARY_DIR}/src/compute_hamming_threshold_128
 #        DESTINATION ${CMAKE_BINARY_DIR}/bin) 

#install( PROGRAMS ${CMAKE_BINARY_DIR}/src/he_sf_root_sift
 #        DESTINATION ${CMAKE_BINARY_DIR}/bin) 

#install( PROGRAMS ${CMAKE_BINARY_DIR}/src/he_sf_root_sift_128
 #        DESTINATION ${CMAKE_BINARY_DIR}/bin) 

