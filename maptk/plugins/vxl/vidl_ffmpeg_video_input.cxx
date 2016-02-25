/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Implementation file for video input using VXL methods.
 */

#include "vidl_ffmpeg_video_input.h"

#include <vital/types/timestamp.h>
#include <vital/exceptions/io.h>
#include <vital/exceptions/video.h>
#include <vital/video_metadata/convert_metadata.h>
#include <vital/vital_foreach.h>

#include <vital/klv/klv_data.h>

#include <maptk/plugins/vxl/image_container.h>

#include <vidl/vidl_ffmpeg_istream.h>
#include <vidl/vidl_convert.h>

#include <mutex>
#include <memory>

namespace kwiver {
namespace maptk {
namespace vxl {

// ------------------------------------------------------------------
// Private implementation class
class vidl_ffmpeg_video_input::priv
{
public:
  /// Constructor
  priv()
    : config_start_frame( -1 ),
      config_stop_after_frame( -1 ),
      config_time_source( "pts" ),
      have_frame( false ),
      at_eov( false ),
      have_frame_time( false ),
      have_metadata( false ),
      pts_of_meta_ts( 0.0 ),
      meta_ts( 0 ),
      frame_time( 0 ),
      frame_number( 1 )
  { }


  vidl_ffmpeg_istream video_stream;
  vital::logger_handle_t logger; // for logging in priv methods

  // Configuration values
  unsigned int config_start_frame;
  unsigned int config_stop_after_frame;
  std::string  config_time_source;

  // local state
  bool have_frame;
  bool at_eov;
  bool have_frame_time;
  bool have_metadata;

  double pts_of_meta_ts;            // probably seconds
  vital::timestamp::time_t meta_ts; // time in usec

  // used to create timestamp output
  vital::timestamp::time_t frame_time; // usec
  vital::timestamp::frame_t frame_number;

  std::string video_path; // name of video we opened

  std::deque<uint8_t> md_buffer; // working buffer for metadata stream
  std::vector< kwiver::vital::video_metadata_sptr > metadata_collection; // current collection

  kwiver::vital::convert_metadata converter; // metadata converter object

  static std::mutex s_open_mutex;
  static const std::string misp;


  // ==================================================================
  /*
   * @brief Process metadata byte stream.
   *
   * This method adds the supplied bytes to the metadata buffer and
   * then tests to see if we have collected enough bytes to make a
   * full metadata packet. If not, then we just return, leaving any
   * current metadata as it was.
   *
   * If a complete klv packet has been received, it is processed and
   * the existing metadata collection is added to the current list of
   * metadata packets.
   *
   * @param curr_md Stream of metadata bytes.
   *
   * @return \b true if there was enough metadata to process.
   */
  bool process_metadata( std::deque<uint8_t> const& curr_md )
  {
    bool retval(false);

    // Add new metadata to the end of current metadata stream
    md_buffer.insert(md_buffer.end(), curr_md.begin(), curr_md.end());
    kwiver::vital::klv_data klv_packet;

    // If we have collected enough of the stream to make a KLV packet
    while ( klv_pop_next_packet( md_buffer, klv_packet ) )
    {
      kwiver::vital::video_metadata_sptr meta( new kwiver::vital::video_metadata );

      converter.convert( klv_packet, *(meta) );

      // If the metadata was even partially decided, then add to the list.
      if ( ! meta->empty() )
      {
        kwiver::vital::timestamp ts;
        ts.set_frame( this->frame_number );

        if ( this->have_frame_time )
        {
          ts.set_time_usec( this->frame_time );
        }

        meta->set_timestamp( ts );
        this->metadata_collection.push_back( meta );

        // indicate we have found
        retval = true;
      } // end valid metadata packet.
    } // end while

    return retval;
  }


// ==================================================================
//Extract the time stamp from the buffer
  bool convertMISPmicrosectime( std::vector< unsigned char > const& buf, int64_t& ts )
  {
    enum MISP_time_code { MISPmicrosectime = 0,
                          FLAG = 16,
                          MSB_0,
                          MSB_1,
                          IGNORE_0,
                          MSB_2,
                          MSB_3,
                          IGNORE_1,
                          MSB_4,
                          MSB_5,
                          IGNORE_2,
                          MSB_6,
                          MSB_7,
                          IGNORE_3,
                          MISP_NUM_ELEMENTS };

    //Check that the tag is the first thing in buf
    for ( size_t i = 0; i < FLAG; i++ )
    {
      if ( buf[i] != misp[i] )
      {
        return false;
      }
    }

    if ( buf.size() >= MISP_NUM_ELEMENTS )
    {
      ts = 0;

      ts |= static_cast< int64_t > ( buf[MSB_7] );
      ts |= static_cast< int64_t > ( buf[MSB_6] ) << 8;
      ts |= static_cast< int64_t > ( buf[MSB_5] ) << 16;
      ts |= static_cast< int64_t > ( buf[MSB_4] ) << 24;

      ts |= static_cast< int64_t > ( buf[MSB_3] ) << 32;
      ts |= static_cast< int64_t > ( buf[MSB_2] ) << 40;
      ts |= static_cast< int64_t > ( buf[MSB_1] ) << 48;
      ts |= static_cast< int64_t > ( buf[MSB_0] ) << 56;

      return true;
    }

    return false;
  } // convertMISPmicrosectime


// ------------------------------------------------------------------
  /*
   * @brief Initialize timestamp for video.
   *
   * This method initializes the timestamp at the start of a video,
   * since we need a timestamp for the first frame. It scans ahead in
   * the input stream until it gets a time marker of the specified
   * type.
   *
   * @return \b true if timestamp has been determined.
   */
  bool init_timestamp()
  {
    bool retval( true );

    meta_ts = 0.0;
    if ( ! this->video_stream.advance() )
    {
      return false;
    }

    have_metadata = this->video_stream.has_metadata();

    // Determine which option has been selected to generate frame time;
    if ( config_time_source == "pts" )
    {
      retval = pts_time();
    }
    else if ( config_time_source == "misp" )
    {
      retval = misp_time();
    }
    else if ( config_time_source == "klv0601" )
    {
      retval = klv_time( kwiver::vital::video_metadata::MISB_0601 );
    }
    else if ( config_time_source == "klv0104" )
    {
      retval = klv_time( kwiver::vital::video_metadata::MISB_0104 );
    }
    else // assume "none"
    {
      have_frame_time = false;
    }

    if (meta_ts != 0)
    {
      pts_of_meta_ts = video_stream.current_pts();
      frame_time = meta_ts;
    }

    // Tried seeking to the beginning but some videos don't
    // want to seek, even to the start.  So reload the video.
    {
      std::lock_guard< std::mutex > lock( s_open_mutex );
      video_stream.open( this->video_path ); //Calls close on current video
    }

    if ( ! video_stream.advance() )
    {
      retval = false;
    }

    // Clear any old metadata
    metadata_collection.clear();

    return retval;
  } // init_timestamp


// ------------------------------------------------------------------
  bool pts_time()
  {
    LOG_DEBUG( this->logger, "Using pts for timestamp: " << video_stream.current_pts() );
    pts_of_meta_ts = video_stream.current_pts();

    // If the pts is zero, then assume that there is no time from video source
    if ( 0.0 != pts_of_meta_ts )
    {
      have_frame_time = false;
    }

    return true;
  }


// ------------------------------------------------------------------
  bool misp_time()
  {
    do
    {
      std::vector< unsigned char > pkt_data = video_stream.current_packet_data();

      //Check if the data packet has enough bytes for the MISPmicrosectime packet
      if ( pkt_data.size() < misp.length() + 13 )
      {
        continue;
      }

      bool found;
      size_t ts_location = std::string::npos;
      size_t last = pkt_data.size() - misp.size();
      for ( size_t i = 0; i <= last; i++ )
      {
        found = true;
        for ( size_t j = 0; j < misp.size(); j++ )
        {
          if ( pkt_data[i + j] != misp[j] )
          {
            found = false;
            break;
          }
        }

        if ( found )
        {
          ts_location = i;
          break;
        }
      } // end for

      if ( ( std::string::npos != ts_location ) && ( ( ts_location + misp.length() + 13 ) < pkt_data.size() ) )
      {
        std::vector< unsigned char > MISPtime_buf( pkt_data.begin() + ts_location,
                                                   pkt_data.begin() + ts_location + misp.length() + 13 );
        int64_t ts = 0;

        convertMISPmicrosectime( MISPtime_buf, ts );
        meta_ts = ts; // in usec
        LOG_DEBUG( this->logger, "Found MISP frame time:" << meta_ts );

        have_frame_time = true;
      }

    }
    while ( meta_ts == 0.0 && video_stream.advance() );

    return true;
  } // misp_time


// ------------------------------------------------------------------
  bool klv_time( std::string type )
  {
    meta_ts = 0;
    do
    {
      // skip ahead until we get some metadata
      if ( video_stream.current_metadata().empty() )
      {
        continue;
      }

      //It might be more accurate to get the second unique timestamp instead of the first
      std::deque< vxl_byte > curr_md = video_stream.current_metadata();
      if (process_metadata( curr_md ) )
      {
        // A metadata collection was created
        // check to see if it is of the desired type.
        std::string collection_type;
        VITAL_FOREACH( auto meta, this->metadata_collection)
        {
          // Test to see if the collection is from the specified standard (0104/0601)
          if (meta->has( kwiver::vital::VITAL_META_METADATA_ORIGIN ) )
          {
            collection_type = meta->find( kwiver::vital::VITAL_META_METADATA_ORIGIN ).as_string();

            if (type == collection_type)
            {
              if (meta->has( kwiver::vital::VITAL_META_UNIX_TIMESTAMP ) )
              {
                // Get unix timestamp and convert to usec
                meta_ts = meta->find( kwiver::vital::VITAL_META_UNIX_TIMESTAMP ).as_uint64() * 1e6;

                LOG_DEBUG( this->logger, "Found initial " << type << " timestamp: " << meta_ts );

                have_frame_time = true;
                return true;
              } // has time element
            } // correct metadata type
          } // has metadata origin
        } // foreach over all metadata packets
      } // end if processed metadata collection
    }
    while ( meta_ts == 0 && video_stream.advance() );

    return false;
  } // klv_time

}; // end of internal class.

// static open interlocking mutex
std::mutex vidl_ffmpeg_video_input::priv::s_open_mutex;
const std::string vidl_ffmpeg_video_input::priv::misp( "MISPmicrosectime" );


// ==================================================================
vidl_ffmpeg_video_input
::vidl_ffmpeg_video_input()
  : d( new priv() )
{
  attach_logger( "video_input" ); // get appropriate logger
  d->logger = this->m_logger;
}


/// copy constructor
vidl_ffmpeg_video_input
::vidl_ffmpeg_video_input( vidl_ffmpeg_video_input const& other )
  : d( new priv() )
{
  attach_logger( "video_input" ); // get appropriate logger
  d->logger = this->m_logger;

  // Copy configuration values only
  d->config_start_frame      = other.d->config_start_frame;
  d->config_stop_after_frame = other.d->config_stop_after_frame;
}


vidl_ffmpeg_video_input
::~vidl_ffmpeg_video_input()
{
  d->video_stream.close( );
}


// ------------------------------------------------------------------
// Get this algorithm's \link vital::config_block configuration block \endlink
vital::config_block_sptr
vidl_ffmpeg_video_input
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config = vital::algo::video_input::get_configuration();

  config->set_value( "start_at_frame", d->config_start_frame,
                     "Frame number (from 1) to start processing video input. "
                     "Default is to start at the beginning of the video." );

  config->set_value( "config_stop_after_frame", d->config_stop_after_frame,
                     "Number of frames to supply. Default is all frames after start frame." );

  config->set_value( "time_source", d->config_time_source,
                     "Source for frame time information. Valid options are \"none\", \"pts\", \"misp\", \"klv0601\", \"klv0104\".\n"
                     "Where:\n"
                     "    none - do not supply frame time\n"
                     "    pts - presentation time stamp. Not all videos can supply this.\n"
                     "    misp - use frame embedded time stamps.\n"
                     "    klv0601 - use klv 0601 format metadata for frame time\n"
                     "    klv0104 - use klv 0104 format metadata for frame time" );

  return config;
}


// ------------------------------------------------------------------
// Set this algorithm's properties via a config block
void
vidl_ffmpeg_video_input
::set_configuration(vital::config_block_sptr in_config)
{
  // Starting with our generated vital::config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.

  vital::config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);

  d->config_start_frame = config->get_value<vital::timestamp::frame_t>(
    "start_at_frame", d->config_start_frame );

  d->config_stop_after_frame = config->get_value<vital::timestamp::frame_t>(
    "stop_after_frame", d->config_stop_after_frame );

  d->config_time_source = config->get_value<std::string>(
    "time_source", d->config_time_source );
}


// ------------------------------------------------------------------
bool
vidl_ffmpeg_video_input
::check_configuration(vital::config_block_sptr config) const
{
  bool retcode(true); // assume success

  // validate time source
  std::string source = config->get_value<std::string>( "time_source", d->config_time_source );
  if (source != "none"
      && source != "pts"
      && source != "misp"
      && source != "klv0601"
      && source != "klv0104")
  {
    LOG_ERROR( m_logger, "time_source must be one of none, pts, misp, klv0601, klv0104" );
    retcode = false;
  }

  // validate start frame
  vital::timestamp::frame_t frame =  config->get_value<vital::timestamp::frame_t>( "start_at_frame");
  // -1 is not set, otherwise must be greater than 1
  if (frame != -1 && frame < 0 )
  {
    LOG_ERROR( m_logger, "start_at_frame must be greater than 0" );
    retcode = false;
  }

  return retcode;
}


// ------------------------------------------------------------------
void
vidl_ffmpeg_video_input
::open( std::string video_name )
{
  d->video_path = video_name;

  // If the open succeeds, it will already have read the first frame.
  // avcodec_open2 which is called by open is not thread safe so we need to lock.
  {
    std::lock_guard< std::mutex > lock( d->s_open_mutex );
    if( ! d->video_stream.open( video_name ) )
    {
      // Throw exception
      throw kwiver::vital::file_not_found_exception( video_name, "Open failed" );
    }
  }

  d->at_eov = false;
  d->frame_number = 1;

  if ( d->config_start_frame != unsigned(-1) &&  d->config_start_frame > 1 )
  {
    // move stream to specified frame number
    d->video_stream.seek_frame( d->config_start_frame );
  }

  // We already have required frame
  d->have_frame = true;

  if( ! d->init_timestamp() )  // will call advance()
  {
    LOG_ERROR( m_logger, "Failed to initialize the timestamp for: " << d->video_path );
    throw kwiver::vital::video_stream_exception( "could not initialize timestamp" );
  }

  // Set traits
  set_trait(vital::algo::video_input_traits::HAS_EOV, true );
  set_trait(vital::algo::video_input_traits::HAS_FRAME_NUMBERS, true );
  set_trait(vital::algo::video_input_traits::HAS_FRAME_TIME, d->have_frame_time  );
  set_trait(vital::algo::video_input_traits::HAS_METADATA, d->have_metadata  );
}


// ------------------------------------------------------------------
void
vidl_ffmpeg_video_input
::close()
{
  d->video_stream.close();

  d->have_frame = false;
  d->at_eov = false;
  d->have_frame_time = false;
  d->have_metadata = false;
  d->frame_time = 0;
  d->frame_number = 1;
}


// ------------------------------------------------------------------
bool
vidl_ffmpeg_video_input
::next_frame( kwiver::vital::image_container_sptr& frame,
              kwiver::vital::timestamp& ts,
              uint32_t timeout )
{
  if (d->at_eov)
  {
    return false;
  }

  // is stream open?
  if ( ! d->video_stream.is_open() )
  {
    throw vital::file_not_read_exception( d->video_path, "Video not open" );
  }

  // Sometimes we already have the frame available.
  if ( d->have_frame )
  {
    d->have_frame = false;
  }
  else
  {
    if( ! d->video_stream.advance() )
    {
      d->at_eov = true;
      return false;
    }
  }

  unsigned int frame_num = d->video_stream.frame_number();
  if( d->config_stop_after_frame < frame_num )
  {
    d->at_eov = true;  // logical end of file
    return false;
  }

  // We succeed in the step if we can convert the frame to RGB.
  vil_image_view<vxl_byte> img;
  vidl_frame_sptr vidl_frame = d->video_stream.current_frame();
  bool result = vidl_convert_to_view( *vidl_frame,
                                      img,
                                      VIDL_PIXEL_COLOR_RGB );

  if ( ! result )
  {
    throw kwiver::vital::video_stream_exception( "could not convert image to vidl format" );
  }

  frame = vital::image_container_sptr( new vxl::image_container( img ) );

  // ---- Calculate time stamp ----
  // Metadata packets may not exist for each frame, so use the diff in
  // presentation time stamps to foward the first metadata time stamp.
  double pts_diff = ( d->video_stream.current_pts() - d->pts_of_meta_ts ) * 1e6;
  d->frame_time = d->meta_ts + pts_diff;
  d->frame_number = d->video_stream.frame_number();

  // We don't always have all components of a timestamp, so start with
  // an invalid TS and add the data we have.
  ts.set_invalid();
  ts.set_frame( d->frame_number );

  if ( d->have_frame_time )
  {
    ts.set_time_usec( d->frame_time );
  }

  // ---- process metadata ---
  d->metadata_collection.clear(); // erase old metadata packets

  std::deque<uint8_t> curr_md = d->video_stream.current_metadata();
  if (curr_md.size() > 0 )
  {
    // will manage metadata collection object.
    d->process_metadata( curr_md );
  }

  return true;
}


// ------------------------------------------------------------------
std::vector< kwiver::vital::video_metadata_sptr >
vidl_ffmpeg_video_input
::frame_metadata()
{
  return d->metadata_collection;
}


// ------------------------------------------------------------------
bool
vidl_ffmpeg_video_input
::end_of_video() const
{
  return d->at_eov;
}


// ------------------------------------------------------------------
bool
vidl_ffmpeg_video_input
::good() const
{
  return d->video_stream.is_valid();
}

} } } // end namespace
