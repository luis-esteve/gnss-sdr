/*!
 * \file gps_l1_ca_ars_dpe.cc
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


#include "gps_l1_ca_ars_dpe.h"
#include <glog/logging.h>
#include "configuration_interface.h"
//#include "gps_l1_ca_pvt_cc.h"

using google::LogMessage;

GpsL1CaArsDpe::GpsL1CaArsDpe(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
                role_(role),
                in_streams_(in_streams),
                out_streams_(out_streams),
                queue_(queue)
{
    // dump parameters
    std::string default_dump_filename = "./dpe_pvt.dat";
    std::string default_nmea_dump_filename = "./nmea_dpe_pvt.nmea";
    std::string default_nmea_dump_devname = "/dev/tty1";
    DLOG(INFO) << "role " << role;
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);

    // output rate
    int output_rate_ms;
    output_rate_ms = configuration->property(role + ".output_rate_ms", 500);
    // display rate
    int display_rate_ms;
    display_rate_ms = configuration->property(role + ".display_rate_ms", 500);
    // NMEA Printer settings
    bool flag_nmea_tty_port;
    flag_nmea_tty_port = configuration->property(role + ".flag_nmea_tty_port", false);
    std::string nmea_dump_filename;
    nmea_dump_filename = configuration->property(role + ".nmea_dump_filename", default_nmea_dump_filename);
    std::string nmea_dump_devname;
    nmea_dump_devname = configuration->property(role + ".nmea_dump_devname", default_nmea_dump_devname);
    // make PVT object
    dpe_ = gps_l1_ca_make_ars_dpe_cc(in_streams_, queue_, dump_, dump_filename_, output_rate_ms, display_rate_ms, flag_nmea_tty_port, nmea_dump_filename, nmea_dump_devname);
    DLOG(INFO) << "pvt(" << dpe_->unique_id() << ")";
}


GpsL1CaArsDpe::~GpsL1CaArsDpe()
{}

void GpsL1CaArsDpe::connect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	// Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}

void GpsL1CaArsDpe::disconnect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	// Nothing to disconnect
}

gr::basic_block_sptr GpsL1CaArsDpe::get_left_block()
{
    return dpe_;
}

gr::basic_block_sptr GpsL1CaArsDpe::get_right_block()
{
    return dpe_;
}

