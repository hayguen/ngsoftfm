///////////////////////////////////////////////////////////////////////////////////
// SoftFM - Software decoder for FM broadcast radio with stereo support          //
//                                                                               //
// Copyright (C) 2019 Hayati Ayguen <h_ayguen@web.de>                            //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
/////////////////////////////////////////////////////////////////////////////////// 

#ifndef SOFTFM_WAVEFILESOURCE_H
#define SOFTFM_WAVEFILESOURCE_H

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <thread>

#include "Source.h"

class WaveFileSource : public Source
{
public:

    static const int default_block_length = 4 * 1024;

    /** Open pseudo device. */
    WaveFileSource();

    /** Close pseudo device. */
    virtual ~WaveFileSource();

    virtual bool configure(std::string configuration);

    /** Return current sample frequency in Hz. */
    virtual std::uint32_t get_sample_rate();

    /** Return device current center frequency in Hz. */
    virtual std::uint32_t get_frequency();

    /** Print current parameters specific to device type */
    virtual void print_specific_parms();

    virtual bool start(DataBuffer<IQSample>* samples, std::atomic_bool *stop_flag);
    virtual bool stop();

    /** Return true if the device is OK, return false if there is an error. */
    /* this is called BEFORE configure() ! */
    virtual operator bool() const
    {
        return m_error.empty();
    }

    /** Return a list of supported devices. */
    static void get_device_names(std::vector<std::string>& devices);

private:
    /**
     * Configure RTL-SDR tuner and prepare for streaming.
     *
     * sample_rate  :: desired sample rate in Hz.
     * frequency    :: desired center frequency in Hz.
     * tuner_gain   :: desired tuner gain in 0.1 dB, or INT_MIN for auto-gain.
     * block_length :: preferred number of samples per block.
     *
     * Return true for success, false if an error occurred.
     */
    bool configure(const std::string & filename, uint32_t tune_freq, int block_length );

    /** Return a list of supported tuner gain settings in units of 0.1 dB. */
    //std::vector<int> get_tuner_gains();

    /** Return current tuner gain in units of 0.1 dB. */
    //int get_tuner_gain();

    /**
     * Fetch a bunch of samples from the device.
     *
     * This function must be called regularly to maintain streaming.
     * Return true for success, false if an error occurred.
     */
    bool get_samples(IQSampleVector *samples);

    static void run();

    bool                m_file_open;
    struct meta_t
    {
        FILE          * file;
        uint32_t        srate;
        uint32_t        freq;
        int             bps;
        int             nchan;
        uint32_t        nframes;
        int16_t         fmttag;
        
        int             inputFmt;
    } m_meta;

    int                 m_block_length;
    std::atomic_bool    m_eof_reached;

    std::thread         *m_thread;
    static WaveFileSource *m_this;

    /* read up to 64K samples */
    uint8_t inpBuffer[2 * 65536 * sizeof(int32_t)];

};

#endif
