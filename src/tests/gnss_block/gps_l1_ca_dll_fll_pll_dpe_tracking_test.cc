/*!
 * \file gps_l1_ca_dll_fll_pll_dpe_tracking_test.cc
 * \brief  This class implements a tracking test for 
 *  GPS_L1_CA_DLL_FLL_PLL_DPE_Tracking implementation
 *  based on some input parameters.
 * \author Luis Esteve, 2015. luis.esteve.elfau(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2015  (see AUTHORS file for a list of contributors)
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


#include <ctime>
#include <iostream>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_dll_fll_pll_dpe_tracking.h"


class GpsL1CaDllFllPllDpeTrackingInternalTest: public ::testing::Test
{
protected:
    GpsL1CaDllFllPllDpeTrackingInternalTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CaDllFllPllDpeTrackingInternalTest()
    {}

    void init();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    concurrent_queue<int> channel_internal_queue;
    bool stop;
    int message;
};


void GpsL1CaDllFllPllDpeTrackingInternalTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 11;

    config->set_property("GNSS-SDR.internal_fs_hz", "4000000");
    config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("Tracking.dump", "true");
    config->set_property("Tracking.dump_filename", "../data/dpe_tracking_ch_");
    config->set_property("Tracking.implementation", "GPS_L1_CA_DLL_FLL_PLL_DPE_Tracking");
    config->set_property("Tracking.order", "2");
    config->set_property("Tracking.pll_bw_hz", "30.0");
    config->set_property("Tracking.fll_bw_hz", "100.0");
    config->set_property("Tracking.dll_bw_hz", "2.0");
    config->set_property("Tracking.number_of_correlators", "3");
    config->set_property("Tracking.correlators_space_chips", "0.5");
    

}



TEST_F(GpsL1CaDllFllPllDpeTrackingInternalTest, Instantiate)
{

    init();
    auto tracking = factory->GetBlock(config, "Tracking", "GPS_L1_CA_DLL_FLL_PLL_DPE_Tracking", 1, 1, queue);
    EXPECT_STREQ("GPS_L1_CA_DLL_FLL_PLL_DPE_Tracking", tracking->implementation().c_str());
}


TEST_F(GpsL1CaDllFllPllDpeTrackingInternalTest, ConnectAndRun)
{
    int fs_in = 4000000;
    int nsamples = 40000000;
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    init();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");

    // Example using smart pointers and the block factory
    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking", "GPS_L1_CA_DLL_FLL_PLL_DPE_Tracking", 1, 1, queue);
    std::shared_ptr<GpsL1CaDllFllPllDpeTracking> tracking = std::dynamic_pointer_cast<GpsL1CaDllFllPllDpeTracking>(trk_);

    ASSERT_NO_THROW( {
        tracking->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        tracking->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        tracking->set_channel_queue(&channel_internal_queue);
    }) << "Failure setting channel_internal_queue." << std::endl;


    ASSERT_NO_THROW( {
        tracking->connect(top_block);
        gr::analog::sig_source_c::sptr source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);

    }) << "Failure connecting the blocks of tracking test." << std::endl;

    tracking->start_tracking();

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec *1000000 + tv.tv_usec;
        top_block->run();   //Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec *1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    std::cout <<  "Processed " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;
}



TEST_F(GpsL1CaDllFllPllDpeTrackingInternalTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    int num_samples = 8000000;
    unsigned int skiphead_sps = 4000000;
    init();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");

    bool test_dump = config->property("Tracking.dump",false);

    std::cout << "test_dump set to " << test_dump << std::endl;

    // Example using smart pointers and the block factory
    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking", "GPS_L1_CA_DLL_FLL_PLL_DPE_Tracking", 1, 1, queue);
    std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);

    gnss_synchro.Acq_delay_samples = 524;
    gnss_synchro.Acq_doppler_hz = -1680;
    gnss_synchro.Acq_samplestamp_samples = 0;

    ASSERT_NO_THROW( {
        tracking->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        tracking->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        tracking->set_channel_queue(&channel_internal_queue);
    }) << "Failure setting channel_internal_queue." << std::endl;

    ASSERT_NO_THROW( {
        tracking->connect(top_block);
    }) << "Failure connecting tracking to the top_block." << std::endl;

    ASSERT_NO_THROW( {
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex),file_name,false);
        gr::blocks::skiphead::sptr skip_head = gr::blocks::skiphead::make(sizeof(gr_complex), skiphead_sps);
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), num_samples, queue);
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, skip_head, 0);
        top_block->connect(skip_head, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);
    }) << "Failure connecting the blocks of tracking test." << std::endl;

    tracking->start_tracking();

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec *1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec *1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    std::cout <<  "Tracked " << num_samples << " samples in " << (end - begin) << " microseconds" << std::endl;
}