/*!
 * \file gps_l1_ca_dll_fll_pll_multicorrelator_tracking.cc
 * \brief Interface of an adapter of a code DLL + carrier FLL/PLL tracking
 * loop with multiple correlators for GPS L1 C/A to a TrackingInterface
 *
 * \author Luis Esteve, 2015. luis.esteve.elfau(at)gmail.com
 *
 * This is the interface of a code Delay Locked Loop (DLL) + carrier
 * Phase Locked Loop (PLL) helped with a carrier Frequency Locked Loop (FLL)
 * according to the algorithms described in:
 * E.D. Kaplan and C. Hegarty, Understanding GPS. Principles and
 * Applications, Second Edition, Artech House Publishers, 2005.
 * The number of correlators can be adjusted in the configuration file in
 * order to use it in Direct Position Estimation
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

#include "gps_l1_ca_dll_fll_pll_multicorrelator_tracking.h"
#include <glog/logging.h>
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include <boost/lexical_cast.hpp>
#include <iostream> // only for debugging


using google::LogMessage;

GpsL1CaDllFllPllMulticorrelatorTracking::GpsL1CaDllFllPllMulticorrelatorTracking(
        ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams, unsigned int
        out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
        role_(role),
        in_streams_(in_streams),
        out_streams_(out_streams),
        queue_(queue)
{
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    int fs_in;
    int vector_length;
    int f_if;
    bool dump;
    std::string dump_filename;
    std::string item_type;
    std::string default_item_type = "gr_complex";
    float pll_bw_hz;
    float fll_bw_hz;
    float dll_bw_hz;
    unsigned int num_oneside_correlators;
    float *correlators_space_chips;
    unsigned int el_index;
    int order;
    bool matlab_enable;
    unsigned int matlab_plot_period;
    item_type = configuration->property(role + ".item_type",default_item_type);
    fs_in = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    f_if = configuration->property(role + ".if", 0);
    dump = configuration->property(role + ".dump", false);
    std::string default_dump_filename = "./track_ch";
    dump_filename = configuration->property(role + ".dump_filename",
            default_dump_filename); //unused!
    order = configuration->property(role + ".order", 2);
    pll_bw_hz = configuration->property(role + ".pll_bw_hz", 50.0);
    fll_bw_hz = configuration->property(role + ".fll_bw_hz", 100.0);
    dll_bw_hz = configuration->property(role + ".dll_bw_hz", 2.0);
    num_oneside_correlators = configuration->property(role + ".number_of_oneside_correlators", 3);
    correlators_space_chips = new float[num_oneside_correlators];
    for (unsigned int i=0; i < num_oneside_correlators; i++)
        {
            correlators_space_chips[i] = configuration->property(role + ".correlators_space_chips_" + boost::lexical_cast<std::string>(i), 0.5);
        }
    std::cout << "Constructor del adapter" << std::endl;
    for (unsigned int i= 0; i < num_oneside_correlators; i++)
        {
            std::cout << " correlators_space_chips[" << i << "] = " << correlators_space_chips[i] << std::endl;
        }
    el_index = configuration->property(role + ".correlators_EL_index", 0);
    std::cout << "EL index = " << el_index << std::endl;
    matlab_enable = configuration->property(role + ".Matlab_enable", false);
    matlab_plot_period = configuration->property(role + ".Matlab_plot_period", 50);

    vector_length = std::round(fs_in / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));

    //################# MAKE TRACKING GNURadio object ###################
    if (item_type.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = gps_l1_ca_dll_fll_pll_multicorrelator_make_tracking_cc(
                    f_if,
                    fs_in,
                    vector_length,
                    queue_,
                    dump,
                    dump_filename,
                    order,
                    fll_bw_hz,
                    pll_bw_hz,
                    dll_bw_hz,
                    num_oneside_correlators,
                    correlators_space_chips,
                    el_index,
                    matlab_enable,
                    matlab_plot_period);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << item_type << " unknown tracking item type.";
        }
    channel_ = 0;
    channel_internal_queue_ = 0;

    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
}


GpsL1CaDllFllPllMulticorrelatorTracking::~GpsL1CaDllFllPllMulticorrelatorTracking()
{}


void GpsL1CaDllFllPllMulticorrelatorTracking::start_tracking()
{
    tracking_->start_tracking();
}

void GpsL1CaDllFllPllMulticorrelatorTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}

void GpsL1CaDllFllPllMulticorrelatorTracking::set_channel_queue(
        concurrent_queue<int> *channel_internal_queue)
{
    channel_internal_queue_ = channel_internal_queue;
    tracking_->set_channel_queue(channel_internal_queue_);
}

void GpsL1CaDllFllPllMulticorrelatorTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    return tracking_->set_gnss_synchro(p_gnss_synchro);
}

void GpsL1CaDllFllPllMulticorrelatorTracking::connect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	//nothing to connect, now the tracking uses gr_sync_decimator
}

void GpsL1CaDllFllPllMulticorrelatorTracking::disconnect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	//nothing to disconnect, now the tracking uses gr_sync_decimator
}

gr::basic_block_sptr GpsL1CaDllFllPllMulticorrelatorTracking::get_left_block()
{
    return tracking_;
}

gr::basic_block_sptr GpsL1CaDllFllPllMulticorrelatorTracking::get_right_block()
{
    return tracking_;
}

