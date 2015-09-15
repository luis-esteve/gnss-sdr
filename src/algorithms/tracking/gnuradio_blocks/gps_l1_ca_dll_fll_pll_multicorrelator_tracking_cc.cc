/*!
 * \file gps_l1_ca_dll_fll_pll_multicorrelator_tracking_cc.cc
 * \brief GNU Radio block of a DLL + carrier FLL/PLL tracking
 * loop with multiple correlators for GPS L1 C/A 
 *
 * \author Luis Esteve, 2015. luis.esteve.elfau(at)gmail.com
 *
 * This is the GNU Radio block  of a code Delay Locked Loop (DLL) + carrier
 * Phase Locked Loop (PLL) helped with a carrier Frequency Locked Loop (FLL)
 * according to the algorithms described in:
 * E.D. Kaplan and C. Hegarty, Understanding GPS. Principles and
 * Applications, Second Edition, Artech House Publishers, 2005.
 * The number of correlators can be adjusted in the configuration file in
 * order to use it in Direct Position Estimation
 *
 * -------------------------------------------------------------------------
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

#include "gps_l1_ca_dll_fll_pll_multicorrelator_tracking_cc.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include "gnss_synchro.h"
#include "gps_sdr_signal_processing.h"
#include "GPS_L1_CA.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "tracking_FLL_PLL_filter.h"
#include "control_message_factory.h"
#include "gnss_flowgraph.h"

/*!
 * \todo Include in definition header file
 */
#define CN0_ESTIMATION_SAMPLES 20
#define MINIMUM_VALID_CN0 25
#define MAXIMUM_LOCK_FAIL_COUNTER 50
#define CARRIER_LOCK_THRESHOLD 0.85

using google::LogMessage;

gps_l1_ca_dll_fll_pll_multicorrelator_tracking_cc_sptr gps_l1_ca_dll_fll_pll_multicorrelator_make_tracking_cc(
        long if_freq,
        long fs_in,
        unsigned
        int vector_length,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump, std::string dump_filename,
        int order,
        float fll_bw_hz,
        float pll_bw_hz,
        float dll_bw_hz,
        unsigned int num_oneside_correlators,
        float *correlators_space_chips,
        unsigned int el_index,
        bool matlab_enable,
        unsigned int matlab_plot_period)
{
//std::cout << "make del bloque gnu" << std::endl;
// for (unsigned int i= 0; i < num_oneside_correlators; i++)
//         {
//             std::cout << " correlators_space_chips[" << i << "] = " << correlators_space_chips[i] << std::endl;
//         }
    return gps_l1_ca_dll_fll_pll_multicorrelator_tracking_cc_sptr(new Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc(if_freq,
            fs_in, vector_length, queue, dump, dump_filename, order, fll_bw_hz, pll_bw_hz,dll_bw_hz,
            num_oneside_correlators, correlators_space_chips, el_index, matlab_enable, matlab_plot_period));
}


void Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = d_vector_length * 2; //set the required available samples in each call
}


Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc::Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump,
        std::string dump_filename,
        int order,
        float fll_bw_hz,
        float pll_bw_hz,
        float dll_bw_hz,
        unsigned int num_oneside_correlators,
        float *correlators_space_chips,
        unsigned int el_index,
        bool matlab_enable,
        unsigned int matlab_plot_period) :
        gr::block("Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    std::cout << "Constructor" << std::endl;
    // initialize internal vars
    d_queue = queue;
    d_dump = dump;

    d_acquisition_gnss_synchro = NULL;

    d_if_freq = static_cast<double>(if_freq);
    d_fs_in = static_cast<double>(fs_in);
    d_vector_length = vector_length;
    d_num_oneside_correlators = num_oneside_correlators;
    d_num_correlators = 2*d_num_oneside_correlators + 1;
    std::cout << "d_num_oneside_correlators = " << d_num_oneside_correlators << std::endl;
    std::cout << "d_num_correlators = " << d_num_correlators << std::endl;
    d_correlators_space_chips = new double[d_num_oneside_correlators];
    d_code_index = new int[d_num_correlators];
    for(unsigned int i = 0; i < d_num_oneside_correlators; i++)
    {
        d_correlators_space_chips[i] = static_cast<double>(correlators_space_chips[i]); // Define correlators distance (in chips)
    }
    d_el_index = el_index;
    std::cout << "d_el_index = " << d_el_index << std::endl;
    d_dump_filename = dump_filename;

    // Initialize tracking variables ==========================================
    d_carrier_loop_filter.set_params(fll_bw_hz, pll_bw_hz,order);

    d_code_loop_filter = Tracking_2nd_DLL_filter(GPS_L1_CA_CODE_PERIOD);
    d_code_loop_filter.set_DLL_BW(dll_bw_hz);

    // Get space for a vector with the C/A code replica sampled 1x/chip
    d_ca_code = static_cast<gr_complex*>(volk_malloc((3*GPS_L1_CA_CODE_LENGTH_CHIPS) * sizeof(gr_complex), volk_get_alignment()));

    // Get space for the resampled replicas of multicorrelator
    d_code = static_cast<gr_complex*>(volk_malloc(d_num_correlators*2*d_vector_length * sizeof(gr_complex), volk_get_alignment()));

    // space for carrier wipeoff and signal baseband vectors
    d_carr_sign = static_cast<gr_complex*>(volk_malloc(2*d_vector_length * sizeof(gr_complex), volk_get_alignment()));

    // correlator outputs (scalar)
    d_Early = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Late = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    
    d_output = static_cast<gr_complex*>(volk_malloc(d_num_correlators*sizeof(gr_complex), volk_get_alignment()));
    
    // sample synchronization
    d_sample_counter = 0;
    d_acq_sample_stamp = 0;
    d_last_seg = 0;// this is for debug output only
    d_code_phase_samples = 0;
    d_enable_tracking = false;
    d_current_prn_length_samples = static_cast<int>(d_vector_length);

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_threshold = CARRIER_LOCK_THRESHOLD;
    
    systemName["G"] = std::string("GPS");
    systemName["R"] = std::string("GLONASS");
    systemName["S"] = std::string("SBAS");
    systemName["E"] = std::string("Galileo");
    systemName["C"] = std::string("Compass");

    d_channel_internal_queue = 0;
    //d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    d_acq_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz = 0;
    d_code_freq_hz = 0.0;
    d_rem_carr_phase = 0.0;
    d_rem_code_phase_samples = 0.0;
    d_acq_code_phase_samples = 0;
    d_acq_carrier_doppler_hz = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_acc_code_phase_samples = 0;
    d_FLL_discriminator_hz = 0.0;
    d_pull_in = false;
    d_FLL_wait = 1;

    d_matlab_enable = matlab_enable;

    if(d_matlab_enable)
        {
            d_ep=engOpen("\0"); //Start the Matlab engine.
        }

    d_matlab_plot_period = matlab_plot_period;
    
    d_matlab_count = 0;

    for (unsigned int i= 0; i < d_num_oneside_correlators; i++)
        {
            std::cout << " d_correlators_space_chips[" << i << "] = " << d_correlators_space_chips[i] << std::endl;
        }
}




void Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc::start_tracking()
{
    // std::cout << "Start tracking" << std::endl;
    /*
     *  correct the code phase according to the delay between acq and trk
     */
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp =  d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    long int acq_trk_diff_samples;
    float acq_trk_diff_seconds;
    acq_trk_diff_samples = static_cast<long int>(d_sample_counter) - static_cast<long int>(d_acq_sample_stamp);
    acq_trk_diff_seconds = static_cast<double>(acq_trk_diff_samples) / d_fs_in;
    //doppler effect
    // Fd=(C/(C+Vr))*F
    double radial_velocity;
    radial_velocity = (GPS_L1_FREQ_HZ + d_acq_carrier_doppler_hz) / GPS_L1_FREQ_HZ;
    // new chip and prn sequence periods based on acq Doppler
    double T_chip_mod_seconds;
    double T_prn_mod_seconds;
    double T_prn_mod_samples;
    d_code_freq_hz = radial_velocity * GPS_L1_CA_CODE_RATE_HZ;
    T_chip_mod_seconds = 1 / d_code_freq_hz;
    T_prn_mod_seconds = T_chip_mod_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
    T_prn_mod_samples = T_prn_mod_seconds * d_fs_in;
    d_current_prn_length_samples = round(T_prn_mod_samples);

    double T_prn_true_seconds = GPS_L1_CA_CODE_LENGTH_CHIPS / GPS_L1_CA_CODE_RATE_HZ;
    double T_prn_true_samples = T_prn_true_seconds * d_fs_in;
    double T_prn_diff_seconds;
    T_prn_diff_seconds = T_prn_true_seconds - T_prn_mod_seconds;
    double N_prn_diff;
    N_prn_diff = acq_trk_diff_seconds / T_prn_true_seconds;
    double corrected_acq_phase_samples, delay_correction_samples;
    corrected_acq_phase_samples = fmod((d_acq_code_phase_samples + T_prn_diff_seconds * N_prn_diff * d_fs_in), T_prn_true_samples);

    if (corrected_acq_phase_samples < 0)
        {
            corrected_acq_phase_samples = T_prn_mod_samples + corrected_acq_phase_samples;
        }
    delay_correction_samples = d_acq_code_phase_samples - corrected_acq_phase_samples;
    d_acq_code_phase_samples = corrected_acq_phase_samples;

    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    // DLL/PLL filter initialization
    d_carrier_loop_filter.initialize(d_acq_carrier_doppler_hz);
    d_FLL_wait = 1;

    // generate 3 local replicas reference (1 sample per chip)
    int code_length_chips = static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS);
    gps_l1_ca_code_gen_complex(&d_ca_code[0], d_acquisition_gnss_synchro->PRN, 0);
    gps_l1_ca_code_gen_complex(&d_ca_code[code_length_chips], d_acquisition_gnss_synchro->PRN, 0);
    gps_l1_ca_code_gen_complex(&d_ca_code[2*code_length_chips], d_acquisition_gnss_synchro->PRN, 0);

    d_carrier_lock_fail_counter = 0;
    d_Prompt_prev = 0;
    d_rem_code_phase_samples = 0;
    d_rem_carr_phase = 0;
    d_FLL_discriminator_hz = 0;
    d_acc_carrier_phase_rad = 0;

    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0,1);

    // DEBUG OUTPUT
    std::cout << "Tracking start on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Starting tracking of satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;

    // enable tracking Gnss_Satellite(systemName[&d_acquisition_gnss_synchro->System], d_acquisition_gnss_synchro->PRN)
    d_pull_in = true;
    d_enable_tracking = true;

    LOG(INFO) << "PULL-IN Doppler [Hz]= " << d_carrier_doppler_hz
              << " Code Phase correction [samples]=" << delay_correction_samples
              << " PULL-IN Code Phase [samples]= " << d_acq_code_phase_samples << std::endl;
}





void Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc::update_local_code()
{
    // std::cout << "Update local code" << std::endl;
    double tcode_chips;
    double rem_code_phase_chips;
    double code_phase_step_chips;
    int early_late_spc_samples[d_num_oneside_correlators];
    unsigned int correlator_loop_length_samples;

    int associated_chip_index;
    int code_length_chips = static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS);
    code_phase_step_chips = d_code_freq_hz / d_fs_in;
    rem_code_phase_chips = d_rem_code_phase_samples * (d_code_freq_hz / d_fs_in);
    // unified loop for E, P, L code vectors
    tcode_chips = -rem_code_phase_chips;
    // Alternative EPL code generation (40% of speed improvement!)
    for(unsigned int i =0; i < d_num_oneside_correlators; i++)
        {
            early_late_spc_samples[i] = round(d_correlators_space_chips[i]/code_phase_step_chips);
        }
    
    // for(unsigned int i =0; i < d_num_oneside_correlators; i++)
    //     {
    //         std::cout << "early_late_spc_samples[" << i << "] = " << early_late_spc_samples[i] << std::endl;
    //     }

    correlator_loop_length_samples = d_current_prn_length_samples + early_late_spc_samples[d_num_oneside_correlators-1]*2;

    for (unsigned int i = 0; i < correlator_loop_length_samples; i++)
        {
            associated_chip_index = code_length_chips + round(fmod(tcode_chips - d_correlators_space_chips[d_num_oneside_correlators-1], code_length_chips));
            d_code[i] = d_ca_code[associated_chip_index];
            tcode_chips = tcode_chips + code_phase_step_chips;
        }

    d_code_index[d_num_oneside_correlators] = 0;

    for(unsigned int i =0; i < d_num_oneside_correlators; i++)
        {
            d_code_index[i] = -early_late_spc_samples[d_num_oneside_correlators-1-i];
        }

    for(unsigned int i =0; i < d_num_oneside_correlators; i++)
        {
            d_code_index[d_num_oneside_correlators+i+1] = early_late_spc_samples[i];
        }
    // for(unsigned int i =0; i < d_num_correlators; i++)
    //     {
    //         std::cout << "d_code_index[" << i << "] = " << d_code_index[i] << std::endl;
    //     }
}





void Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc::update_local_carrier()
{
    // std::cout << "Update local carrier" << std::endl;
    double phase, phase_step;
    phase_step = GPS_TWO_PI * d_carrier_doppler_hz / d_fs_in;
    phase = d_rem_carr_phase;
    for(int i = 0; i < d_current_prn_length_samples; i++)
        {
            d_carr_sign[i] = gr_complex(cos(phase), -sin(phase));
            phase += phase_step;
        }
    d_rem_carr_phase = fmod(phase, GPS_TWO_PI);
    d_acc_carrier_phase_rad = d_acc_carrier_phase_rad + phase;
}



Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc::~Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc()
{
    d_dump_file.close();

    volk_free(d_ca_code);
    volk_free(d_code);
    volk_free(d_carr_sign);
    volk_free(d_output);
    volk_free(d_Early);
    volk_free(d_Prompt);
    volk_free(d_Late);

    delete[] d_correlators_space_chips;
    delete[] d_code_index;
    delete[] d_Prompt_buffer;
    
    if(d_matlab_enable)
        {  
            engClose(d_ep);  //Close Matlab engine.
        }
}


int Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // std::cout << "General work" << std::endl;
    double code_error_chips = 0;
    double code_error_filt_chips = 0;
    double correlation_time_s = 0;
    double PLL_discriminator_hz = 0;
    double carr_nco_hz = 0;
    // get the sample in and out pointers
    const gr_complex* in = (gr_complex*) input_items[0];     // block input samples pointer
    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0]; // block output streams pointer

    d_Prompt_prev = *d_Prompt; // for the FLL discriminator

    // std::cout << "Antes del primer if" << std::endl;

    if (d_enable_tracking == true)
        {
            // std::cout << "d_enable_tracking == true" << std::endl;
            // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
            Gnss_Synchro current_synchro_data;
            // Fill the acquisition data
            current_synchro_data = *d_acquisition_gnss_synchro;
            /*
             * Receiver signal alignment
             */
            if (d_pull_in == true)
                {
                    // std::cout << "d_pull_in == true" << std::endl;                
                    int samples_offset;
                    double acq_trk_shif_correction_samples;
                    int acq_to_trk_delay_samples;
                    acq_to_trk_delay_samples = d_sample_counter - d_acq_sample_stamp;
                    acq_trk_shif_correction_samples = d_current_prn_length_samples - fmod(static_cast<double>(acq_to_trk_delay_samples), static_cast<double>(d_current_prn_length_samples));
                    samples_offset = round(d_acq_code_phase_samples + acq_trk_shif_correction_samples);
                    // /todo: Check if the sample counter sent to the next block as a time reference should be incremented AFTER sended or BEFORE
                    d_sample_counter = d_sample_counter + samples_offset; //count for the processed samples
                    d_pull_in = false;
                    consume_each(samples_offset); //shift input to perform alignment with local replica

                    // make an output to not stop the rest of the processing blocks
                    current_synchro_data.Prompt_I = 0.0;
                    current_synchro_data.Prompt_Q = 0.0;
                    current_synchro_data.Tracking_timestamp_secs = static_cast<double>(d_sample_counter) / d_fs_in;
                    current_synchro_data.Carrier_phase_rads = 0.0;
                    current_synchro_data.Code_phase_secs = 0.0;
                    current_synchro_data.CN0_dB_hz = 0.0;
                    for (unsigned int i = 0; i < d_num_correlators; i++)
                        {
                            current_synchro_data.Multi_correlation[i] = d_output[i]; // TODO: Check if it is necessary or better =0
                        }
                    current_synchro_data.Flag_valid_tracking = false;
                    current_synchro_data.Flag_valid_pseudorange = false;

                    *out[0] = current_synchro_data;

                    return 1;
                }
            // std::cout << "Antes de updates" << std::endl;
            update_local_code();
            update_local_carrier();

            // perform the correlations

            d_correlator.Carrier_wipeoff_and_multiEPL_volk(d_current_prn_length_samples,
                    in,
                    d_carr_sign,
                    d_code,
                    d_num_correlators,
                    d_code_index,
                    d_output);

            *d_Early = d_output[d_num_oneside_correlators-d_el_index-1];
            *d_Prompt = d_output[d_num_oneside_correlators];
            *d_Late = d_output[d_num_oneside_correlators+d_el_index+1];


            // check for samples consistency (this should be done before in the receiver / here only if the source is a file)
            if (std::isnan((*d_Prompt).real()) == true or std::isnan((*d_Prompt).imag()) == true )// or std::isinf(in[i].real())==true or std::isinf(in[i].imag())==true)
                {
                    const int samples_available = ninput_items[0];
                    d_sample_counter = d_sample_counter + samples_available;
                    LOG(WARNING) << "Detected NaN samples at sample number " << d_sample_counter;
                    consume_each(samples_available);

                    // make an output to not stop the rest of the processing blocks
                    current_synchro_data.Prompt_I = 0.0;
                    current_synchro_data.Prompt_Q = 0.0;
                    current_synchro_data.Tracking_timestamp_secs = static_cast<double>(d_sample_counter) / d_fs_in;
                    current_synchro_data.Carrier_phase_rads = 0.0;
                    current_synchro_data.Code_phase_secs = 0.0;
                    current_synchro_data.CN0_dB_hz = 0.0;
                    current_synchro_data.Flag_valid_tracking = false;
                    current_synchro_data.Flag_valid_pseudorange = false;

                    *out[0] = current_synchro_data;

                    return 1;
                }

            /*
             * DLL, FLL, and PLL discriminators
             */
            // Compute DLL error
            code_error_chips = dll_nc_e_minus_l_normalized(*d_Early, *d_Late);
            // Compute DLL filtered error
            code_error_filt_chips = d_code_loop_filter.get_code_nco(code_error_chips);

            //compute FLL error
            correlation_time_s = (static_cast<double>(d_current_prn_length_samples)) / d_fs_in;
            if (d_FLL_wait == 1)
                {
                    d_Prompt_prev = *d_Prompt;
                    d_FLL_wait = 0;
                }
            else
                {
                    d_FLL_discriminator_hz = fll_four_quadrant_atan(d_Prompt_prev, *d_Prompt, 0, correlation_time_s) / GPS_TWO_PI;
                    d_Prompt_prev = *d_Prompt;
                    d_FLL_wait = 1;
                }

            // Compute PLL error
            PLL_discriminator_hz = pll_cloop_two_quadrant_atan(*d_Prompt) / GPS_TWO_PI;

            /*
             * DLL and FLL+PLL filter and get current carrier Doppler and code frequency
             */
            carr_nco_hz = d_carrier_loop_filter.get_carrier_error(d_FLL_discriminator_hz, PLL_discriminator_hz, correlation_time_s);
            d_carrier_doppler_hz = d_if_freq + carr_nco_hz;

            d_code_freq_hz = GPS_L1_CA_CODE_RATE_HZ + (((d_carrier_doppler_hz + d_if_freq) * GPS_L1_CA_CODE_RATE_HZ) / GPS_L1_FREQ_HZ);

            /*!
             * \todo Improve the lock detection algorithm!
             */
            // ####### CN0 ESTIMATION AND LOCK DETECTORS ######
            if (d_cn0_estimation_counter < CN0_ESTIMATION_SAMPLES)
                {
                    // fill buffer with prompt correlator output values
                    d_Prompt_buffer[d_cn0_estimation_counter] = *d_Prompt;
                    d_cn0_estimation_counter++;
                }
            else
                {
                    d_cn0_estimation_counter = 0;
                    //d_CN0_SNV_dB_Hz = gps_l1_ca_CN0_SNV(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, d_fs_in);
                    d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, d_fs_in, GPS_L1_CA_CODE_LENGTH_CHIPS);

                    d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES);
                    // ###### TRACKING UNLOCK NOTIFICATION #####
                    if (d_carrier_lock_test < d_carrier_lock_threshold or d_CN0_SNV_dB_Hz < MINIMUM_VALID_CN0)
                        {
                            d_carrier_lock_fail_counter++;
                        }
                    else
                        {
                            if (d_carrier_lock_fail_counter > 0) d_carrier_lock_fail_counter--;
                        }
                    if (d_carrier_lock_fail_counter > MAXIMUM_LOCK_FAIL_COUNTER)
                        {
                            std::cout << "Loss of lock in channel " << d_channel << "!" << std::endl;
                            LOG(INFO) << "Loss of lock in channel " << d_channel << "!";
                            std::unique_ptr<ControlMessageFactory> cmf(new ControlMessageFactory());
                            if (d_queue != gr::msg_queue::sptr())
                                {
                                    d_queue->handle(cmf->GetQueueMessage(d_channel, 2));
                                }
                            d_carrier_lock_fail_counter = 0;
                            d_enable_tracking = false; // TODO: check if disabling tracking is consistent with the channel state machine
                        }
                }

            // ########## DEBUG OUTPUT
            /*!
             *  \todo The stop timer has to be moved to the signal source!
             */
            // debug: Second counter in channel 0
            if (d_channel == 0)
                {
                    if (floor(d_sample_counter/d_fs_in) != d_last_seg)
                        {
                            d_last_seg = floor(d_sample_counter / d_fs_in);
                            std::cout << "Current input signal time = " << d_last_seg << " [s]" << std::endl;
                            LOG(INFO) << "Tracking CH " << d_channel <<  ": Satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN)  << ", CN0 = " << d_CN0_SNV_dB_Hz << " [dB-Hz]";
                        }
                }
            else
                {
                    if (floor(d_sample_counter/d_fs_in) != d_last_seg)
                        {
                            d_last_seg = floor(d_sample_counter/d_fs_in);
                            LOG(INFO) << "Tracking CH " << d_channel <<  ": Satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN)  << ", CN0 = " << d_CN0_SNV_dB_Hz << " [dB-Hz]";
                        }
                }

            //predict the next loop PRN period length prediction
            double T_chip_seconds;
            double T_prn_seconds;
            double T_prn_samples;
            double K_blk_samples;
            T_chip_seconds = 1 / static_cast<double>(d_code_freq_hz);
            T_prn_seconds = T_chip_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
            T_prn_samples = T_prn_seconds * d_fs_in;

            float code_error_filt_samples;
            code_error_filt_samples = T_prn_seconds * code_error_filt_chips * T_chip_seconds * static_cast<double>(d_fs_in); //[seconds]
            d_acc_code_phase_samples = d_acc_code_phase_samples + code_error_filt_samples;

            K_blk_samples = T_prn_samples + d_rem_code_phase_samples + code_error_filt_samples;
            d_current_prn_length_samples = round(K_blk_samples); //round to a discrete sample
            //d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error

            // ########### Output the tracking data to navigation and PVT ##########
            current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).real());
            current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).imag());
            // Tracking_timestamp_secs is aligned with the PRN start sample
            //current_synchro_data.Tracking_timestamp_secs = ((double)d_sample_counter + (double)d_current_prn_length_samples + (double)d_rem_code_phase_samples) / (double)d_fs_in;
            current_synchro_data.Tracking_timestamp_secs = (static_cast<double>(d_sample_counter) + static_cast<double>(d_rem_code_phase_samples))/static_cast<double>(d_fs_in);
            d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample
            // This tracking block aligns the Tracking_timestamp_secs with the start sample of the PRN, Code_phase_secs=0
            current_synchro_data.Code_phase_secs = 0;
            current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_rad;
            current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
            current_synchro_data.CN0_dB_hz = d_CN0_SNV_dB_Hz;
            for (unsigned int i = 0; i < 25; i++)
            {
                 if(i < d_num_correlators)
                 {
                    current_synchro_data.Multi_correlation[i] = d_output[i];
                 }
                 else
                 {
                    current_synchro_data.Multi_correlation[i] = gr_complex(0,0);
                 } 
            }
            current_synchro_data.Flag_valid_tracking = true;
            current_synchro_data.Flag_valid_pseudorange = false;
            *out[0] = current_synchro_data;
        }
    else
        {
            // std::cout << "d_enable_tracking == false" << std::endl;
			// ########## DEBUG OUTPUT (TIME ONLY for channel 0 when tracking is disabled)
			/*!
			 *  \todo The stop timer has to be moved to the signal source!
			 */
			// stream to collect cout calls to improve thread safety
			std::stringstream tmp_str_stream;
			if (floor(d_sample_counter / d_fs_in) != d_last_seg)
			{
				d_last_seg = floor(d_sample_counter / d_fs_in);

				if (d_channel == 0)
				{
					// debug: Second counter in channel 0
					tmp_str_stream << "Current input signal time = " << d_last_seg << " [s]" << std::endl << std::flush;
					std::cout << tmp_str_stream.rdbuf() << std::flush;
				}
			}
            *d_Early  = gr_complex(0,0);
            *d_Prompt = gr_complex(0,0);
            *d_Late   = gr_complex(0,0);

            // std::cout << "Antes de llenar d_output de ceros" << std::endl;
            // std::cout << "d_num_correlators = " <<  d_num_correlators << std::endl;

            for (unsigned int i = 0; i < d_num_correlators; i++)
            {
                d_output[i] = gr_complex(0,0);
            }

            Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0]; //block output streams pointer
            d_acquisition_gnss_synchro->Flag_valid_tracking = false;
            d_acquisition_gnss_synchro->Flag_valid_pseudorange = false;
            *out[0] = *d_acquisition_gnss_synchro;
        }
    // std::cout << "sale del else" << std::endl;
    if(d_dump)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file

            float prompt_I;
            float prompt_Q;
            float tmp_E, tmp_P, tmp_L;
            float tmp_float;
            double tmp_double;
            prompt_I = (*d_Prompt).real();
            prompt_Q = (*d_Prompt).imag();
            tmp_E = std::abs<float>(*d_Early);
            tmp_P = std::abs<float>(*d_Prompt);
            tmp_L = std::abs<float>(*d_Late);

            try
            {
                    // EPR
                    d_dump_file.write((char*)&tmp_E, sizeof(float));
                    d_dump_file.write((char*)&tmp_P, sizeof(float));
                    d_dump_file.write((char*)&tmp_L, sizeof(float));
                    // PROMPT I and Q (to analyze navigation symbols)
                    d_dump_file.write((char*)&prompt_I, sizeof(float));
                    d_dump_file.write((char*)&prompt_Q, sizeof(float));
                    // PRN start sample stamp
                    d_dump_file.write((char*)&d_sample_counter, sizeof(unsigned long int));
                    // accumulated carrier phase
                    tmp_float = (float)d_acc_carrier_phase_rad;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));

                    // carrier and code frequency
                    tmp_float = (float)d_carrier_doppler_hz;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));
                    tmp_float = (float)d_code_freq_hz;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));

                    //PLL commands
                    tmp_float = (float)PLL_discriminator_hz;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));
                    tmp_float = (float)carr_nco_hz;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));

                    //DLL commands
                    tmp_float = (float)code_error_chips;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));
                    tmp_float = (float)code_error_filt_chips;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));

                    // CN0 and carrier lock test
                    tmp_float = (float)d_CN0_SNV_dB_Hz;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));
                    tmp_float = (float)d_carrier_lock_test;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));

                    // AUX vars (for debug purposes)
                    tmp_float = (float)d_rem_code_phase_samples;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));
                    tmp_double = (double)(d_sample_counter + d_current_prn_length_samples);
                    d_dump_file.write((char*)&tmp_double, sizeof(double));
            }
            catch (std::ifstream::failure e)
            {
                    LOG(INFO) << "Exception writing trk dump file "<< e.what() << std::endl;
            }
        }


    if(d_matlab_enable)
        {
            // MATLAB Representation

            float tmp_out[d_num_correlators];
            for (unsigned int i = 0; i < d_num_correlators; i++)
                {
                    tmp_out[i] = std::abs<float>(d_output[i]);
                }

            float tmp_E, tmp_P, tmp_L;
            tmp_E = std::abs<float>(*d_Early);
            tmp_P = std::abs<float>(*d_Prompt);
            tmp_L = std::abs<float>(*d_Late);
            
            if(d_matlab_count >= d_matlab_plot_period)
            {  
            //si el modulo es cero, entonces es multiplo  
                if(d_matlab_count%d_matlab_plot_period == 0)
                {
                    
                    //Defined mxArray, the array is one line of real numbers, N columns.
                    mxArray *m_corr = mxCreateDoubleMatrix(d_num_correlators, 1, mxREAL);
                    double *tmp_corr = new double [d_num_correlators];
            
                    // Copy the correlators value to a vector 
                    for (unsigned int i = 0; i < d_num_correlators; i++)
                    {
                        tmp_corr[i] = static_cast<double>(tmp_out[i]);
                    }

                    //Copy the c ++ array of value to the corresponding mxArray 
                    memcpy(mxGetPr(m_corr), tmp_corr, d_num_correlators*sizeof(double));

                    //MxArray array will be written to the Matlab workspace 
                    engPutVariable(d_ep, "corr", m_corr);
                    
                    mxArray *m_count = mxCreateDoubleMatrix(1, 1, mxREAL);
                    double tmp_m_count;
                    tmp_m_count = static_cast<double>(d_matlab_count);
                    memcpy(mxGetPr(m_count), &tmp_m_count, sizeof(double));
                    engPutVariable(d_ep, "mat_count", m_count);

                    mxArray *m_channel = mxCreateDoubleMatrix(1, 1, mxREAL);
                    double tmp_m_channel;
                    tmp_m_channel = static_cast<double>(d_channel);
                    memcpy(mxGetPr(m_channel), &tmp_m_channel, sizeof(double));
                    engPutVariable(d_ep, "mat_channel", m_channel);

                    

                    mxArray *m_prn = mxCreateDoubleMatrix(1, 1, mxREAL);
                    double tmp_m_prn;
                    tmp_m_prn = static_cast<double>(d_acquisition_gnss_synchro->PRN);
                    memcpy(mxGetPr(m_prn), &tmp_m_prn, sizeof(double));
                    engPutVariable(d_ep, "mat_prn", m_prn);

                    mxArray *m_index = mxCreateDoubleMatrix(d_num_correlators, 1, mxREAL);
                    double *tmp_m_index = new double [d_num_correlators];
                    for (unsigned int i = 0; i < d_num_correlators; i++)
                    {
                        tmp_m_index[i] = static_cast<double>(d_code_index[i]);
                    }
                    
                    memcpy(mxGetPr(m_index), tmp_m_index, d_num_correlators*sizeof(double));
                    engPutVariable(d_ep, "mat_index", m_index);

                    mxArray *m_epl_corr = mxCreateDoubleMatrix(3, 1, mxREAL);
                    double *tmp_m_epl_corr = new double [3];
                    tmp_m_epl_corr[0] = static_cast<double>(tmp_E);
                    tmp_m_epl_corr[1] = static_cast<double>(tmp_P);
                    tmp_m_epl_corr[2] = static_cast<double>(tmp_L);
                    memcpy(mxGetPr(m_epl_corr), tmp_m_epl_corr, 3*sizeof(double));
                    engPutVariable(d_ep, "epl_corr", m_epl_corr);


                    mxArray *m_epl_index = mxCreateDoubleMatrix(3, 1, mxREAL);
                    double *tmp_m_epl_index = new double [3];
                    tmp_m_epl_index[0] = static_cast<double>(d_code_index[d_num_oneside_correlators-d_el_index-1]);
                    tmp_m_epl_index[1] = static_cast<double>(d_code_index[d_num_oneside_correlators]);
                    tmp_m_epl_index[2] = static_cast<double>(d_code_index[d_num_oneside_correlators+d_el_index+1]);
                    memcpy(mxGetPr(m_epl_index), tmp_m_epl_index, 3*sizeof(double));
                    engPutVariable(d_ep, "epl_index", m_epl_index);

                   //Matlab engine sends drawing commands. 
                    engEvalString(d_ep, "plot(mat_index, corr,'-.s'); grid on; title(sprintf('Channel: %u \t PRN = %u \t Number of codes = %u',mat_channel,mat_prn,mat_count)); hold on;");  
                    engEvalString(d_ep, "plot(epl_index, epl_corr,'rs'); hold off; xlabel('samples'); ylabel('Correlators');");            
                    delete tmp_corr;
                    delete tmp_m_index;
                    mxDestroyArray(m_count);
                    mxDestroyArray(m_channel);
                    mxDestroyArray(m_prn);
                    mxDestroyArray(m_index);
                    mxDestroyArray(m_corr);
                    mxDestroyArray(m_epl_index);
                    mxDestroyArray(m_epl_corr);
                }
            }
            d_matlab_count++;
        }
    consume_each(d_current_prn_length_samples); // this is necessary in gr::block derivates
    d_sample_counter += d_current_prn_length_samples; //count for the processed samples
    return 1; //output tracking result ALWAYS even in the case of d_enable_tracking==false
}


void Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc::set_channel(unsigned int channel)
{
    d_channel = channel;
    LOG(INFO) << "Tracking Channel set to " << d_channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Tracking dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                    }
                    catch (std::ifstream::failure e)
                    {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                    }
                }
        }
}



void Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc::set_channel_queue(concurrent_queue<int> *channel_internal_queue)
{
    d_channel_internal_queue = channel_internal_queue;
}

void Gps_L1_Ca_Dll_Fll_Pll_Multicorrelator_Tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro=p_gnss_synchro;
}
