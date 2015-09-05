/*!
 * \file gps_l1_ca_ars_dpe.h
 * \brief Interface of an adapter of a DPE block to a GNSSBlockInterface
 * 
 * \author Luis Esteve, 2015. luis.esteve.elfau@gmail.com 
 *
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



#ifndef GPS_L1_CA_ARS_DPE_H_
#define GPS_L1_CA_ARS_DPE_H_

#include <string>
#include <gnuradio/msg_queue.h>
#include "gnss_block_interface.h"
#include "gps_l1_ca_ars_dpe_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements an Interface for DPE block
 */
class GpsL1CaArsDpe : public GNSSBlockInterface
{
public:
    GpsL1CaArsDpe(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams,
            boost::shared_ptr<gr::msg_queue> queue);

    virtual ~GpsL1CaArsDpe();

    std::string role()
    {
        return role_;
    }

    //!  Returns "GPS_L1_CA_PVT"
    std::string implementation()
    {
        return "GPS_L1_CA_PVT";
    }

    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

    // void reset()
    // {
    //     return;
    // }

    //! All blocks must have an item_size() function implementation. Returns sizeof(gr_complex)
    size_t item_size()
    {
        return sizeof(gr_complex);
    }

private:
    gps_l1_ca_ars_dpe_cc_sptr dpe_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /* GPS_L1_CA_ARS_DPE_H_ */
