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

#include "WaveFileSource.h"
#include <climits>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <unistd.h>

#include "util.h"
#include "parsekv.h"
#include "waveread.h"


WaveFileSource *WaveFileSource::m_this = 0;

// Open pseudo device.
WaveFileSource::WaveFileSource() :
    m_file_open(false),
    m_block_length(default_block_length),
    m_thread(0)
{
    m_meta.file = nullptr;
    m_this = this;
}


// Close pseudo device.
WaveFileSource::~WaveFileSource()
{
    if (m_file_open)
        fclose(m_meta.file);
    m_meta.file = nullptr;
    m_this = nullptr;
}

bool WaveFileSource::configure(std::string configurationStr)
{
fprintf(stderr, "configure('%s')\n", configurationStr.c_str());
    int block_length = default_block_length;
    uint32_t tune_freq = 0;
    std::string filename;
#if 0
    filename = configurationStr;
#else
    namespace qi = boost::spirit::qi;
    std::string::iterator begin = configurationStr.begin();
    std::string::iterator end = configurationStr.end();
    parsekv::key_value_sequence<std::string::iterator> p;
    parsekv::pairs_type m;

    if (!qi::parse(begin, end, p, m))
    {
        m_error = "Configuration parsing failed\n";
        return false;
    }
    else
    {
        if (m.find("freq") != m.end())
        {
            std::cerr << "WaveFileSource::configure: freq: " << m["freq"] << std::endl;
            tune_freq = atoll(m["freq"].c_str());
fprintf(stderr, "parsed tune_freq %lu\n", (unsigned long)tune_freq);
        }

        if (m.find("blklen") != m.end())
        {
            std::cerr << "WaveFileSource::configure: blklen: " << m["blklen"] << std::endl;
            block_length = atoi(m["blklen"].c_str());
        }

        if (m.find("file") != m.end())
        {
            std::cerr << "WaveFileSource::configure: file: " << m["file"] << std::endl;
            filename = m["file"];
fprintf(stderr, "parsed filename '%s'\n", filename.c_str());
        }
    }

#endif

    return configure(filename, tune_freq, block_length);
}

// Configure RTL-SDR tuner and prepare for streaming.
bool WaveFileSource::configure( const std::string & filename,
                             uint32_t tune_freq,
                             int block_length )
{
    int r;

    m_meta.file = fopen( filename.c_str(), "rb" );
    if (!m_meta.file) {
        fprintf(stderr, "error opening file\n");
        return false;
    }

    r = waveReadHeader(m_meta.file, &m_meta.srate, &m_meta.freq, &m_meta.bps, &m_meta.nchan,
        &m_meta.nframes, &m_meta.fmttag, 1);
    if ( r >= 10 ) {
        fprintf(stderr, "error %d parsing wave header\n", r);
        fclose(m_meta.file);
        m_meta.file = nullptr;
        return false;
    }

    if ( m_meta.nchan != 2 ) {
        fprintf(stderr, "error: quadrature signal with I and Q channel required!, input has %d channels\n", m_meta.nchan);
        fclose(m_meta.file);
        m_meta.file = nullptr;
        return false;
    }

    if ( m_meta.fmttag == 0x0001 && m_meta.bps == 16 ) {
        m_meta.inputFmt = 0;	/* PCM16 */
    }
    else if ( m_meta.fmttag == 0x0003 && m_meta.bps == 32 ) {
        m_meta.inputFmt = 1;	/* FLOAT32 */
    }
    else if ( m_meta.fmttag == 0x0001 && m_meta.bps == 24 ) {
        m_meta.inputFmt = 2;	/* PCM24 */
    }
    else
    {
        fprintf(stderr, "error: unsupported input format tag 0x%04X with %d bits. only 'PCM16' and 'FLOAT32' supported.\n"
        , (unsigned)m_meta.fmttag, m_meta.bps);
        fclose(m_meta.file);
        m_meta.file = nullptr;
        return false;
    }

    // set block length
    m_block_length = (block_length < 1024) ? 1024 :
                     (block_length > 64 * 1024) ? 64 * 1024 :
                     block_length;
    m_block_length -= m_block_length % 1024;
    
    m_confFreq = (tune_freq > 0) ? tune_freq : m_meta.freq;

    m_eof_reached.store(false);
    m_file_open = true;
    return true;
}


// Return current sample frequency in Hz.
uint32_t WaveFileSource::get_sample_rate()
{
    return m_meta.srate;
}

// Return device current center frequency in Hz.
uint32_t WaveFileSource::get_frequency()
{
    return m_meta.freq;
}

void WaveFileSource::print_specific_parms()
{
}


bool WaveFileSource::start(DataBuffer<IQSample>* buf, std::atomic_bool *stop_flag)
{
fputs("start\n", stderr);
    m_buf = buf;
    m_stop_flag = stop_flag;
    
    if (m_thread == 0)
    {
        m_thread = new std::thread(run);
        return true;
    }
    else
    {
        m_error = "Source thread already started";
        return false;
    }
}

bool WaveFileSource::stop()
{
    if (m_thread) 
    {
        m_thread->join();
        delete m_thread;
        m_thread = 0;
    }
    
    return true;
}

void WaveFileSource::run()
{
    IQSampleVector iqsamples;
    const size_t srate = m_this->m_meta.srate;
    const size_t minfill = srate;

    while (!m_this->m_stop_flag->load())
    {
        if ( m_this->m_buf->is_buffer_empty( minfill ) && ! m_this->m_eof_reached.load() ) {
            if ( m_this->get_samples(&iqsamples) )
                m_this->m_buf->push( move(iqsamples) );
            else
                break;
        }
        else
            usleep(10000);  // wait 10 ms
    }
}

// Fetch a bunch of samples from the device.
bool WaveFileSource::get_samples(IQSampleVector *samples)
{
    if (!m_this->m_file_open) {
        return false;
    }

    if (!samples) {
        return false;
    }

    void * pvInp = &inpBuffer[0];
    const size_t numToRead = m_block_length;

    size_t numRead;
    const size_t readErr = waveReadFrames(m_meta.file, pvInp, numToRead, 0, &numRead);

    if ( numRead != numToRead )
        fprintf(stderr, "Error: reading %lu delivered %lu frames\n", (unsigned long)numToRead, (unsigned long)numRead );
    if ( readErr ) {
        fprintf(stderr, "Error reading samples - short read\n");
    }
    if ( !numRead ) {
        m_buf->push_end();
        fprintf(stderr, "Error: 0 samples => End-Of-Signal reached\n");
        m_eof_reached.store(true);
        return false;
    }

    samples->resize(m_block_length);
    IQSample * pOut = &(*samples)[0];

    if ( m_meta.inputFmt == 0 ) {
        const int16_t *ai = (const int16_t*)pvInp;
        const IQSample::value_type scale = IQSample::value_type(1.0) / IQSample::value_type(32768);
        int i = 0;
        for ( size_t k = 0; k < numRead; ++k, i+=2 )
        {
            pOut[k] = IQSample( IQSample::value_type( ai[i] ) * scale,
                                IQSample::value_type( ai[i+1] ) * scale );
        }
    }
    else if ( m_meta.inputFmt == 1 ) {
        const float *ai = (const float*)pvInp;
        int i = 0;
        for ( size_t k = 0; k < numRead; ++k, i+=2 )
        {
            pOut[k] = IQSample( IQSample::value_type( ai[i] ),
                                IQSample::value_type( ai[i+1] ) );
        }
    }
    else if ( m_meta.inputFmt == 2 ) {
        // input is PCM24
        const uint8_t *au = (const uint8_t*)pvInp;
        const IQSample::value_type scale = IQSample::value_type(1.0) / IQSample::value_type(2147483648.0);
        int i = 0;
        for ( size_t k = 0; k < numRead; ++k, i+=6 )
        {
            // assume LITTLE ENDIAN!!!
            int32_t valLeft  = ( uint32_t(au[i+2]) << 24 ) | ( uint32_t(au[i+1]) << 16 ) | ( uint32_t(au[i+0]) << 8 );
            int32_t valRight = ( uint32_t(au[i+5]) << 24 ) | ( uint32_t(au[i+4]) << 16 ) | ( uint32_t(au[i+3]) << 8 );
            pOut[k] = IQSample( IQSample::value_type( valLeft ) * scale,
                                IQSample::value_type( valRight ) * scale );
        }
    }

    return true;
}


// Return a list of supported devices.
void WaveFileSource::get_device_names(std::vector<std::string>& devices)
{
    devices.clear();
    std::string devName("dummy");
    devices.push_back(devName);
}

/* end */
