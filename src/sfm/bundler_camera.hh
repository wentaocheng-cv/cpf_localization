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

#ifndef BUNDLER_CAMERA_HH
#define BUNDLER_CAMERA_HH

/**
 * Class to model a Camera in a Bundler reconstruction.
 *
 * author: Torsten Sattler (tsattler@cs.rwth-aachen.de)
 * date  : 09-26-2011
**/
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <stdint.h>

/**
 * Class for storing a camera consisting of a rotation matrix,
 * a translation vector, a focal length value and two parameters 
 * accounting for radial distortion
 *
 * Author: Torsten Sattler <tsattler@cs.rwth-aachen.de>
 * Date: 08-02-2010 
**/

class bundler_camera
{
  public:
    //! constructor
    bundler_camera();
        
    //! destructor
    ~bundler_camera();

    Eigen::Matrix<float, 3, 3> rotation;
    Eigen::Vector3d translation;
    double focal_length;
    double kappa_1, kappa_2;
    uint32_t id;
    int width, height;
    //cheng wentao
    std::vector < int > vote_list;
    //this list preserve 1-to-N to one db image
    std::vector < int > multiple_list;
    std::vector < int > unremove_list;
    std::vector < int > identical_vws;
    int identical_visual_world_nb;
    int valid_corrs_nb;
    int vote_length;
    std::vector < int > point_list;
    std::vector < float > point_pos;
    double probability;
    double avg_hamming_distance;
    uint32_t cover;
    bool covered_flag;
    uint32_t importance;
    uint32_t point_size;
    double pos_x;
    double pos_y;
    double pos_z;
    //added by cwt
    //this is an indicator to tell that whether this image is constructed locally.
    //which means that the points visible in it is very close to the camera center.
    double localness;
    //cheng wentao
    
};

#endif 
