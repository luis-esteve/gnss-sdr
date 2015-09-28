/*!
 * \file gps_l1_ca_ars_dpe_cc.cc
 * \brief DPE block (Direct Position Estimation)
 * \author Luis Esteve, 2015. luis.esteve.elfau@gmail.com 
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

#include "gps_l1_ca_ars_dpe_cc.h"
#include <algorithm>
#include <bitset>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "control_message_factory.h"
#include "gnss_synchro.h"
#include "concurrent_map.h"
#include "sbas_telemetry_data.h"
#include "sbas_ionospheric_correction.h"
#include "dpe_motion_parameters.h"

using google::LogMessage;

extern concurrent_map<Gps_Ephemeris> global_gps_ephemeris_map;
extern concurrent_map<Gps_Iono> global_gps_iono_map;
extern concurrent_map<Gps_Utc_Model> global_gps_utc_model_map;

extern concurrent_queue<Sbas_Raw_Msg> global_sbas_raw_msg_queue;
extern concurrent_map<Sbas_Ionosphere_Correction> global_sbas_iono_map;
extern concurrent_map<Sbas_Satellite_Correction> global_sbas_sat_corr_map;
extern concurrent_map<Sbas_Ephemeris> global_sbas_ephemeris_map;

extern concurrent_queue<Dpe_Motion_Parameters> global_dpe_msg_queue;

gps_l1_ca_ars_dpe_cc_sptr
gps_l1_ca_make_ars_dpe_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, 
    std::string dump_filename, unsigned int num_correlators, unsigned int min_trk_channels, unsigned int min_radius,
    unsigned int max_radius, unsigned int constant_factor, unsigned int num_iter, int output_rate_ms, int display_rate_ms,
    bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname)
{
    return gps_l1_ca_ars_dpe_cc_sptr(new gps_l1_ca_ars_dpe_cc(nchannels, queue, dump, dump_filename, 
        num_correlators, min_trk_channels, min_radius, max_radius, constant_factor, num_iter,
        output_rate_ms, display_rate_ms, flag_nmea_tty_port, nmea_dump_filename, nmea_dump_devname));
}


gps_l1_ca_ars_dpe_cc::gps_l1_ca_ars_dpe_cc(unsigned int nchannels,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump, std::string dump_filename,
        unsigned int num_correlators,
        unsigned int min_trk_channels,
        unsigned int min_radius,
        unsigned int max_radius,
        unsigned int constant_factor,
        unsigned int num_iter,
        int output_rate_ms,
        int display_rate_ms,
        bool flag_nmea_tty_port,
        std::string nmea_dump_filename,
        std::string nmea_dump_devname) :
             gr::block("gps_l1_ca_ars_dpe_cc", gr::io_signature::make(nchannels, nchannels,  sizeof(Gnss_Synchro)),
             gr::io_signature::make(0, 0, sizeof(gr_complex)) )
{
    d_output_rate_ms = output_rate_ms;
    d_display_rate_ms = display_rate_ms;
    d_queue = queue;
    d_dump = dump;
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    std::string dump_ls_pvt_filename = dump_filename;

    //initialize kml_printer
    // std::string kml_dump_filename;
    // kml_dump_filename = d_dump_filename;
    // kml_dump_filename.append(".kml");
    // d_kml_dump = std::make_shared<Kml_Printer>();
    // d_kml_dump->set_headers(kml_dump_filename);

    //initialize nmea_printer
    // d_nmea_printer = std::make_shared<Nmea_Printer>(nmea_dump_filename, flag_nmea_tty_port, nmea_dump_devname);

    d_dump_filename.append("_raw.dat");
    dump_ls_pvt_filename.append("_ls_pvt.dat");

    d_ls_pvt = std::make_shared<gps_l1_ca_ls_pvt>((int)nchannels, dump_ls_pvt_filename, d_dump);

    d_sample_counter = 0;
    d_last_sample_nav_output = 0;
    d_rx_time = 0.0;

    d_num_correlators = num_correlators;
    d_min_trk_channels = min_trk_channels;
    d_min_radius = min_radius;
    d_max_radius = max_radius;
    d_constant_factor = constant_factor;
    d_num_iter = num_iter;
    d_dpe_standby = true;
    d_pull_in = false;

    // b_rinex_header_writen = false;
    // b_rinex_header_updated = false;
    // b_rinex_sbs_header_writen = false;
    // rp = std::make_shared<Rinex_Printer>();

    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "DPE dump enabled Log file: " << d_dump_filename.c_str();
                    }
                    catch (std::ifstream::failure e)
                    {
                            LOG(INFO) << "Exception opening DPE dump file " << e.what();
                    }
                }
        }

    d_matlab_enable = true; // TODO: Put in config file or remove when c++ version of DPE will be finished 
    if(d_matlab_enable)
        {
            d_ep=engOpen("\0"); //Start the Matlab engine.
        }
    
    d_matlab_plot_period = 1000;
    
    d_matlab_count = 0;
}



gps_l1_ca_ars_dpe_cc::~gps_l1_ca_ars_dpe_cc()
{
        if(d_matlab_enable)
        {  
            engClose(d_ep);  //Close Matlab engine.
        }
}


int gps_l1_ca_ars_dpe_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items)
{
    d_sample_counter++;

    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0]; //Get the input pointer
    //Gnss_Synchro **out = (Gnss_Synchro **)  &output_items[0]; //Get the output pointer

    // for (unsigned int i = 0; i < d_nchannels; i++)
    //     {
    //         if (in[i][0].Flag_valid_pseudorange == true)
    //             {
    //                 gnss_pseudoranges_map.insert(std::pair<int,Gnss_Synchro>(in[i][0].PRN, in[i][0])); // store valid pseudoranges in a map
    //             }
    //     }

    Dpe_Motion_Parameters current_dpe_parameters;

    if (d_dpe_standby)
    {
        while(global_dpe_msg_queue.try_pop(current_dpe_parameters))
        {
            std::cout << "PVT message arrival: Position at " << boost::posix_time::to_simple_string(current_dpe_parameters.position_UTC_time)
            << std::endl << "x = " << current_dpe_parameters.pos_x_m << " [m]" << std::endl << "y = " << current_dpe_parameters.pos_y_m 
            << " [m]" << std::endl << "z = " << current_dpe_parameters.pos_z_m  << " [m]" << std::endl << "user clock error = " 
            << current_dpe_parameters.dt_s << " [s]" << std::endl;
            d_dpe_standby = false;
            d_pull_in = true;
        }
    }

    if(!d_dpe_standby)
    {   

//        std::cout << "Entra en el if(!d_dpe_standby)" << std::endl;

        // ############ 1. Empty the message queue ########

        Gnss_Synchro current_gnss_synchro[d_nchannels];
    
        // ############ 2. Read the GNSS SYNCHRO objects from available tracking channels ########

        unsigned int trk_channels = 0;
        for (unsigned int i = 0; i < d_nchannels; i++)
            {

                //Copy the telemetry decoder data to local copy
                current_gnss_synchro[i] = in[i][0];

                //record the word structure in a map for pseudorange computation

                d_rx_time = in[i][0].d_TOW_at_current_symbol; // all the channels have the same RX timestamp (common RX time pseudoranges) // lo necesito?

 
                if (current_gnss_synchro[i].Flag_valid_tracking) //if this channel is in tracking mode
                {
                    trk_channels++;

                }
            }

//        std::cout << "trk_channels = " << trk_channels << std::endl;

        if(d_min_trk_channels > trk_channels)
        {
            d_dpe_standby = true;
            std::cout << "DPE lost of lock, request PVT." << std::endl;
            
        }else
        {
//          std::cout << "Entra en el else." << std::endl;

            // ############ 3. READ EPHEMERIS/UTC_MODE/IONO FROM GLOBAL MAPS ####

            std::map<int,Gps_Ephemeris> gps_ephemeris_map; //!< Map storing new Gps_Ephemeris
            gps_ephemeris_map = global_gps_ephemeris_map.get_map_copy();

            // ############ 4. Execute DPE algorithm
            //std::cout << "Executing DPE. Number of trk channels = " << trk_channels << std::endl;
            if(d_matlab_enable)
            {

                // MATLAB Representation

                if(d_pull_in)
                {
                    //Defined mxArray, the array is one line of real numbers, N columns.
                    mxArray *m_radius = mxCreateDoubleMatrix(2, 1, mxREAL);
                    double tmp_m_radius[2];
                    
                    // Copy the d_min and d_max radius value to a vector 
                    tmp_m_radius[0] = d_min_radius;
                    tmp_m_radius[1] = d_max_radius;

                    //Copy the c ++ array of value to the corresponding mxArray 
                    memcpy(mxGetPr(m_radius), &tmp_m_radius[0], 2*sizeof(double));

                    //MxArray array will be written to the Matlab workspace 
                    engPutVariable(d_ep, "radius", m_radius);
                        
                    mxArray *m_n_iter = mxCreateDoubleMatrix(1, 1, mxREAL);
                    double tmp_m_n_iter;
                    tmp_m_n_iter = static_cast<double>(d_num_iter);
                    memcpy(mxGetPr(m_n_iter), &tmp_m_n_iter, sizeof(double));
                    engPutVariable(d_ep, "n_iter", m_n_iter);

                    mxArray *m_cf = mxCreateDoubleMatrix(1, 1, mxREAL);
                    double tmp_m_cf;
                    tmp_m_cf = static_cast<double>(d_constant_factor);
                    memcpy(mxGetPr(m_cf), &tmp_m_cf, sizeof(double));
                    engPutVariable(d_ep, "cf", m_cf);

                    mxArray *m_pos = mxCreateDoubleMatrix(3, 1, mxREAL);
                    double tmp_m_pos[3];
                    tmp_m_pos[0] = current_dpe_parameters.pos_x_m;
                    tmp_m_pos[1] = current_dpe_parameters.pos_y_m;
                    tmp_m_pos[2] = current_dpe_parameters.pos_z_m;
                    memcpy(mxGetPr(m_pos), &tmp_m_pos, 3*sizeof(double));
                    engPutVariable(d_ep, "pos_0", m_pos);

                    mxArray *m_dt = mxCreateDoubleMatrix(1, 1, mxREAL);
                    double tmp_m_dt;
                    tmp_m_dt = current_dpe_parameters.dt_s;
                    memcpy(mxGetPr(m_dt), &tmp_m_dt, sizeof(double));
                    engPutVariable(d_ep, "dt", m_dt);

                    mxDestroyArray(m_dt);
                    mxDestroyArray(m_pos);
                    mxDestroyArray(m_cf);
                    mxDestroyArray(m_n_iter);
                    mxDestroyArray(m_radius);

                    d_pull_in = false;
                }


                // float tmp_out[d_num_correlators];
                // for (unsigned int i = 0; i < d_num_correlators; i++)
                // {
                //         tmp_out[i] = std::abs<float>(d_output[i]);
                // }

                // float tmp_E, tmp_P, tmp_L;
                // tmp_E = std::abs<float>(*d_Early);
                // tmp_P = std::abs<float>(*d_Prompt);
                // tmp_L = std::abs<float>(*d_Late);
                if(d_matlab_count >= d_matlab_plot_period)
                {  
                    //si el modulo es cero, entonces es multiplo  
                    if(d_matlab_count%d_matlab_plot_period == 0)
                    {
                        mxArray *m_trk_ch = mxCreateDoubleMatrix(1, 1, mxREAL);
                        double tmp_m_trk_ch;
                        tmp_m_trk_ch = static_cast<double>(trk_channels);
                        memcpy(mxGetPr(m_trk_ch), &tmp_m_trk_ch, sizeof(double));
                        engPutVariable(d_ep, "trk_channels", m_trk_ch);

                        mxArray *m_correlators = mxCreateDoubleMatrix(d_num_correlators, trk_channels, mxCOMPLEX);
                        double  *start_of_rp, *start_of_ip;
                        double *tmp_m_real_correlators, *tmp_m_imag_correlators;
                        tmp_m_real_correlators = new double [trk_channels*d_num_correlators];
                        tmp_m_imag_correlators = new double [trk_channels*d_num_correlators];

                        for (unsigned int i, index = 0; i < d_nchannels; i++)
                            {
                            if(current_gnss_synchro[i].Flag_valid_tracking)
                                {
                                    for (unsigned int j = 0; j < d_num_correlators; j++)
                                        {
                                            tmp_m_real_correlators[index] = static_cast<double>(current_gnss_synchro[i].Multi_correlation[j].real());
                                            tmp_m_imag_correlators[index] = static_cast<double>(current_gnss_synchro[i].Multi_correlation[j].imag());
                                            index++;
                                        }
                                        
                                }
                            }

                        // for (unsigned int i; i < trk_channels; i++)
                        //     {
                        //     for (unsigned int j = 0; j < d_num_correlators; j++)
                        //         {
                        //             std::cout << "Correlator ["<< i+1 << "," << j+1 << "] = " << tmp_m_real_correlators[j] << "+j" << tmp_m_imag_correlators[j] << std::endl;
                        //         }
                        //     }

                        for (unsigned int i; i < trk_channels*d_num_correlators; i++)
                            {
                                std::cout << "Correlator ["<< i << "] = " << tmp_m_real_correlators[i] << "+j" << tmp_m_imag_correlators[i] << std::endl;
                            }

                        /* Populate the real part of the created array. */ 
                        start_of_rp = (double *)mxGetPr(m_correlators);
                        memcpy(start_of_rp, tmp_m_real_correlators, trk_channels*d_num_correlators*sizeof(double));
                        /* Populate the imaginary part of the created array. */ 
                        start_of_ip = (double *)mxGetPi(m_correlators);
                        memcpy(start_of_ip, tmp_m_imag_correlators, trk_channels*d_num_correlators*sizeof(double));
                        
                        engPutVariable(d_ep, "correlators", m_correlators);

                        // mxArray *m_epl_corr = mxCreateDoubleMatrix(3, 1, mxREAL);
                        // double *tmp_m_epl_corr = new double [3];
                        // tmp_m_epl_corr[0] = static_cast<double>(tmp_E);
                        // tmp_m_epl_corr[1] = static_cast<double>(tmp_P);
                        // tmp_m_epl_corr[2] = static_cast<double>(tmp_L);
                        // memcpy(mxGetPr(m_epl_corr), tmp_m_epl_corr, 3*sizeof(double));
                        // engPutVariable(d_ep, "epl_corr", m_epl_corr);

                        // mxArray *m_epl_index = mxCreateDoubleMatrix(3, 1, mxREAL);
                        // double *tmp_m_epl_index = new double [3];
                        // tmp_m_epl_index[0] = static_cast<double>(d_code_index[d_num_oneside_correlators-d_el_index-1]);
                        // tmp_m_epl_index[1] = static_cast<double>(d_code_index[d_num_oneside_correlators]);
                        // tmp_m_epl_index[2] = static_cast<double>(d_code_index[d_num_oneside_correlators+d_el_index+1]);
                        // memcpy(mxGetPr(m_epl_index), tmp_m_epl_index, 3*sizeof(double));
                        // engPutVariable(d_ep, "epl_index", m_epl_index);


                        //Matlab engine sends drawing commands. 
                        char buffer[1001]; //TODO: Only for DEBUG
                        buffer[1000] = '\0';
                        engOutputBuffer(d_ep, buffer, 1000);

                        engEvalString(d_ep, "scriptname='/home/luis/dev/gnss-sdr-luis/src/utils/matlab/dpe/ars_dpe'");  
                        engEvalString(d_ep, "run(scriptname)");
                        std::cout << buffer << std::endl;



                        // delete tmp_corr;
                        // delete tmp_m_index;
         
                        // Elimino el vector principal
                        delete[] tmp_m_real_correlators;
                        delete[] tmp_m_imag_correlators;


                        mxDestroyArray(m_trk_ch);
                        mxDestroyArray(m_correlators);
                        // mxDestroyArray(m_epl_index);
                        // mxDestroyArray(m_epl_corr);
                    }
                }
                d_matlab_count++;

            }
        }

        while(global_dpe_msg_queue.try_pop(current_dpe_parameters)) {}




        // ############ 3. Read PVT raw messages directly from queue
    


    // if (global_gps_utc_model_map.size() > 0)
    //     {
    //         // UTC MODEL data is shared for all the GPS satellites. Read always at a locked channel
    //         signed int i = 0;
    //         while(true)
    //             {
    //                 if (in[i][0].Flag_valid_pseudorange == true)
    //                     {
    //                         global_gps_utc_model_map.read(i, d_ls_pvt->gps_utc_model);
    //                         break;
    //                     }
    //                 i++;
    //                 if (i == (signed int)d_nchannels - 1)
    //                     {
    //                         break;
    //                     }
    //             }
    //     }

    // if (global_gps_iono_map.size() > 0)
    //     {
    //         // IONO data is shared for all the GPS satellites. Read always at a locked channel
    //         signed int i = 0;
    //         while(true)
    //             {
    //                 if (in[i][0].Flag_valid_pseudorange == true)
    //                     {
    //                         global_gps_iono_map.read(i, d_ls_pvt->gps_iono);
    //                         break;
    //                     }
    //                 i++;
    //                 if (i == (signed int)d_nchannels - 1)
    //                     {
    //                         break;
    //                     }
    //             }
    //     }



    // ############ 3. Read the que for a valid position ################################




/*    if (gnss_pseudoranges_map.size() > 0 and d_ls_pvt->gps_ephemeris_map.size() > 0)
        {
            // compute on the fly PVT solution
            //mod 8/4/2012 Set the PVT computation rate in this block
            if ((d_sample_counter % d_output_rate_ms) == 0)
                {
                    bool pvt_result;
                    pvt_result = d_ls_pvt->get_PVT(gnss_pseudoranges_map, d_rx_time, d_flag_averaging);
                    if (pvt_result == true)
                        {
                            d_kml_dump->print_position(d_ls_pvt, d_flag_averaging);
                            d_nmea_printer->Print_Nmea_Line(d_ls_pvt, d_flag_averaging);

                            if (!b_rinex_header_writen)
                                {
                                    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                        {
                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, d_rx_time);
                                            rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                                            b_rinex_header_writen = true; // do not write header anymore
                                        }
                                }
                            if(b_rinex_header_writen) // Put here another condition to separate annotations (e.g 30 s)
                                {
                                    // Limit the RINEX navigation output rate to 1/6 seg
                                    // Notice that d_sample_counter period is 1ms (for GPS correlators)
                                    if ((d_sample_counter - d_last_sample_nav_output) >= 6000)
                                        {
                                            rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_ephemeris_map);
                                            d_last_sample_nav_output = d_sample_counter;
                                        }
                                    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                        {
                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, d_rx_time, gnss_pseudoranges_map);
                                        }
                                    if (!b_rinex_header_updated && (d_ls_pvt->gps_utc_model.d_A0 != 0))
                                        {
                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                            rp->update_nav_header(rp->navFile, d_ls_pvt->gps_utc_model, d_ls_pvt->gps_iono);
                                            b_rinex_header_updated = true;
                                        }
                                }
                        }
                }

            // DEBUG MESSAGE: Display position in console output
            if (((d_sample_counter % d_display_rate_ms) == 0) and d_ls_pvt->b_valid_position == true)
                {
                    std::cout << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " UTC is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                              << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]" << std::endl;

                    LOG(INFO) << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " UTC is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                              << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]";

                    LOG(INFO) << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " is HDOP = " << d_ls_pvt->d_HDOP << " VDOP = "
                              << d_ls_pvt->d_VDOP <<" TDOP = " << d_ls_pvt->d_TDOP << " GDOP = " << d_ls_pvt->d_GDOP;
                }
            // MULTIPLEXED FILE RECORDING - Record results to file
            if(d_dump == true)
                {
                    try
                    {
                            double tmp_double;
                            for (unsigned int i = 0; i < d_nchannels ; i++)
                                {
                                    tmp_double = in[i][0].Pseudorange_m;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    tmp_double = 0;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    d_dump_file.write((char*)&d_rx_time, sizeof(double));
                                }
                    }
                    catch (std::ifstream::failure e)
                    {
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                    }
                }
        }*/ 
    }


    consume_each(1); //one by one
    return noutput_items;
}


