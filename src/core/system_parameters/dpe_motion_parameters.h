/*!
 * \file dpe_motion_parameters.h
 * \brief  This class storage the motion parameters of DPE (postion, velocity
 * and time error)
 * \author
 *  Luis Esteve, 2015. luis.esteve.elfau@gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */
#ifndef GNSS_SDR_DPE_MOTION_PARAMETERS_H_
#define GNSS_SDR_DPE_MOTION_PARAMETERS_H_


/*!
 * \brief This is the class that contains the information that is shared
 * by the processing blocks DPE and PVT.
 */
class  Dpe_Motion_Parameters
{
public:

    // User info
    double pos_x_m;
    double pos_y_m;
    double pos_z_m;
    double dt_s;

    double t_s;
};

#endif

