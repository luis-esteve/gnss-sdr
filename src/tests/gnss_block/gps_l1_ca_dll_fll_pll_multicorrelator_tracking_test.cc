/*!
 * \file gps_l1_ca_dll_fll_pll_multicorrelator_tracking_test.cc
 * \brief  This class implements a tracking test for 
 *  GPS_L1_CA_DLL_FLL_PLL_Multicorrelator_Tracking implementation
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
#include "gps_l1_ca_dll_fll_pll_multicorrelator_tracking.h"


class GpsL1CaDllFllPllMulticorrelatorTrackingInternalTest: public ::testing::Test
{
protected:
    GpsL1CaDllFllPllMulticorrelatorTrackingInternalTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CaDllFllPllMulticorrelatorTrackingInternalTest()
    {}

    void config_1();
    void config_2();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    concurrent_queue<int> channel_internal_queue;
    bool stop;
    int message;

    double delay_chips = 0.0;
    double doppler_hz = 0.0;
    unsigned int fs_in = 0;
};


void GpsL1CaDllFllPllMulticorrelatorTrackingInternalTest::config_1()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal,2,0);
    gnss_synchro.PRN = 11;
    fs_in = 4e6;


    config->set_property("GNSS-SDR.internal_fs_hz", "4000000");
    config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("Tracking.dump", "false");
    config->set_property("Tracking.dump_filename", "../data/multicorrelator_tracking_ch_");
    config->set_property("Tracking.implementation", "GPS_L1_CA_DLL_FLL_PLL_Multicorrelator_Tracking");
    config->set_property("Tracking.order", "2");
    config->set_property("Tracking.pll_bw_hz", "30.0");
    config->set_property("Tracking.fll_bw_hz", "100.0");
    config->set_property("Tracking.dll_bw_hz", "2.0");
    config->set_property("Tracking.number_of_oneside_correlators", "4");
    config->set_property("Tracking.correlators_space_chips_0", "0.1");
    config->set_property("Tracking.correlators_space_chips_1", "0.2");
    config->set_property("Tracking.correlators_space_chips_2", "0.3");
    config->set_property("Tracking.correlators_space_chips_3", "0.4");
    config->set_property("Tracking.correlators_EL_index", "1");
    

}

void GpsL1CaDllFllPllMulticorrelatorTrackingInternalTest::config_2()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal,2,0);
    gnss_synchro.PRN = 11;

    fs_in = 4e6;

    delay_chips = 600;
    doppler_hz = 750;

    gnss_synchro.Acq_delay_samples = (delay_chips*static_cast<double>(fs_in)/1.023e6)+10.0;
    gnss_synchro.Acq_doppler_hz = doppler_hz;
    gnss_synchro.Acq_samplestamp_samples = 0;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.item_type", "gr_complex");

    config->set_property("SignalSource.num_satellites", "1");

    config->set_property("SignalSource.system_0", "G");
    config->set_property("SignalSource.PRN_0", "11");
    config->set_property("SignalSource.CN0_dB_0", "44");
    config->set_property("SignalSource.doppler_Hz_0", std::to_string(doppler_hz));
    config->set_property("SignalSource.delay_chips_0", std::to_string(delay_chips));

    config->set_property("SignalSource.noise_flag", "false");
    config->set_property("SignalSource.data_flag", "false");
    config->set_property("SignalSource.BW_BB", "0.97");

    config->set_property("InputFilter.implementation", "Fir_Filter");
    config->set_property("InputFilter.input_item_type", "gr_complex");
    config->set_property("InputFilter.output_item_type", "gr_complex");
    config->set_property("InputFilter.taps_item_type", "float");
    config->set_property("InputFilter.number_of_taps", "11");
    config->set_property("InputFilter.number_of_bands", "2");
    config->set_property("InputFilter.band1_begin", "0.0");
    config->set_property("InputFilter.band1_end", "0.97");
    config->set_property("InputFilter.band2_begin", "0.98");
    config->set_property("InputFilter.band2_end", "1.0");
    config->set_property("InputFilter.ampl1_begin", "1.0");
    config->set_property("InputFilter.ampl1_end", "1.0");
    config->set_property("InputFilter.ampl2_begin", "0.0");
    config->set_property("InputFilter.ampl2_end", "0.0");
    config->set_property("InputFilter.band1_error", "1.0");
    config->set_property("InputFilter.band2_error", "1.0");
    config->set_property("InputFilter.filter_type", "bandpass");
    config->set_property("InputFilter.grid_density", "16");

    config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("Tracking.dump", "false");
    config->set_property("Tracking.dump_filename", "../data/multicorrelator_tracking_ch_");
    config->set_property("Tracking.implementation", "GPS_L1_CA_DLL_FLL_PLL_Multicorrelator_Tracking");
    config->set_property("Tracking.order", "3");
    config->set_property("Tracking.pll_bw_hz", "45.0");
    config->set_property("Tracking.fll_bw_hz", "10.0");
    config->set_property("Tracking.dll_bw_hz", "3.0");
    config->set_property("Tracking.number_of_oneside_correlators", "5");
    config->set_property("Tracking.correlators_space_chips_0", "0.2");
    config->set_property("Tracking.correlators_space_chips_1", "0.4");
    config->set_property("Tracking.correlators_space_chips_2", "0.6");
    config->set_property("Tracking.correlators_space_chips_3", "0.8");
    config->set_property("Tracking.correlators_space_chips_4", "1.0");
    config->set_property("Tracking.correlators_EL_index", "2");
    config->set_property("Tracking.Matlab_enable", "true");
    config->set_property("Tracking.Matlab_plot_period", "100");
}

TEST_F(GpsL1CaDllFllPllMulticorrelatorTrackingInternalTest, Instantiate)
{

    config_1();
    auto tracking = factory->GetBlock(config, "Tracking", "GPS_L1_CA_DLL_FLL_PLL_Multicorrelator_Tracking", 1, 1, queue);
    EXPECT_STREQ("GPS_L1_CA_DLL_FLL_PLL_Multicorrelator_Tracking", tracking->implementation().c_str());
}


TEST_F(GpsL1CaDllFllPllMulticorrelatorTrackingInternalTest, ConnectAndRun)
{
    int nsamples = 40000000;
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    config_1();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");

    // Example using smart pointers and the block factory
    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking", "GPS_L1_CA_DLL_FLL_PLL_Multicorrelator_Tracking", 1, 1, queue);
    std::shared_ptr<GpsL1CaDllFllPllMulticorrelatorTracking> tracking = std::dynamic_pointer_cast<GpsL1CaDllFllPllMulticorrelatorTracking>(trk_);

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



TEST_F(GpsL1CaDllFllPllMulticorrelatorTrackingInternalTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    int num_samples = 40000000;
    config_2();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");

    bool test_dump = config->property("Tracking.dump",false);

    std::cout << "test_dump set to " << test_dump << std::endl;

    // Example using smart pointers and the block factory
    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking", "GPS_L1_CA_DLL_FLL_PLL_Multicorrelator_Tracking", 1, 1, queue);
    std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);

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
        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1, queue);
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), num_samples, queue);
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        signal_source.reset(new GenSignalSource(config.get(), signal_generator, filter, "SignalSource", queue));
        signal_source->connect(top_block);
        top_block->connect(signal_source->get_right_block(), 0, valve, 0);
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