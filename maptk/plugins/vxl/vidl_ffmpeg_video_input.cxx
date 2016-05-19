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
#include <vital/util/tokenize.h>
#include <vital/klv/misp_time.h>
#include <vital/klv/klv_data.h>

#include <maptk/plugins/vxl/image_container.h>

#include <vidl/vidl_ffmpeg_istream.h>
#include <vidl/vidl_convert.h>

#include <mutex>
#include <memory>
#include <vector>
#include <sstream>


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
    : c_start_at_frame( 0 ),
      c_stop_after_frame( 0 ),
      c_time_source( "none" ), // initialization string
      c_time_scan_frame_limit( 100 ),
      d_have_frame( false ),
      d_at_eov( false ),
      d_have_frame_time( false ),
      d_have_abs_frame_time( false ),
      d_have_metadata( false ),
      pts_of_meta_ts( 0.0 ),
      meta_ts( 0 ),
      d_frame_time( 0 ),
      d_frame_number( 1 )
  { }


  vidl_ffmpeg_istream d_video_stream;
  vital::logger_handle_t d_logger; // for logging in priv methods

  // Configuration values
  unsigned int c_start_at_frame;
  unsigned int c_stop_after_frame;
  std::string  c_time_source; // default sources string
  std::vector< std::string >  c_time_source_list;
  int c_time_scan_frame_limit; // number of frames to scan looking for time

  // local state
  bool d_have_frame;
  bool d_at_eov;

  /**
   * This is set to indicate that we can supply a frame time or some
   * form. If this is false, the output timestamp will not have a time
   * set. This also is used to report the HAS_FRAME_TIME capability.
   */
  bool d_have_frame_time;

  /**
   * This is set to indicate that we can supply an absolute frame time
   * rather than a relative frame time. This value is used to report
   * the HAS_ABSOLUTE_FRAME_TIME capability.
   */
  bool d_have_abs_frame_time;

  /**
   * This is set to indicate we can supply video metadata and is used
   * to report the HAS_METADATA capability.
   */
  bool d_have_metadata;

  double pts_of_meta_ts;            // probably seconds
  vital::timestamp::time_t meta_ts; // time in usec

  // used to create timestamp output
  vital::timestamp::time_t d_frame_time; // usec
  vital::timestamp::frame_t d_frame_number;

  std::string video_path; // name of video we opened

  std::deque<uint8_t> md_buffer; // working buffer for metadata stream
  kwiver::vital::video_metadata_vector metadata_collection; // current collection

  kwiver::vital::convert_metadata converter; // metadata converter object

  static std::mutex s_open_mutex;


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
      auto meta = std::make_shared<kwiver::vital::video_metadata>();

      converter.convert( klv_packet, *(meta) );

      // If the metadata was even partially decided, then add to the list.
      if ( ! meta->empty() )
      {
        kwiver::vital::timestamp ts;
        ts.set_frame( this->d_frame_number );

        if ( this->d_have_frame_time )
        {
          ts.set_time_usec( this->d_frame_time );
        }

        meta->set_timestamp( ts );
        this->metadata_collection.push_back( meta );

        // indicate we have found
        retval = true;
      } // end valid metadata packet.
    } // end while

    return retval;
  }


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
  bool init_timestamp( std::string  time_source )
  {
    bool retval( true );

    meta_ts = 0.0;
    if ( ! this->d_video_stream.advance() )
    {
      return false;
    }

    // Determine which option has been selected to generate frame time;
    if ( time_source == "misp" )
    {
      retval = misp_time();
    }
    else if ( time_source == "klv0601" )
    {
      retval = klv_time( kwiver::vital::video_metadata::MISB_0601 );
    }
    else if ( time_source == "klv0104" )
    {
      retval = klv_time( kwiver::vital::video_metadata::MISB_0104 );
    }
    else if ( time_source == "none" )
    {
      d_have_frame_time = false;
      return true;              // optimized return
    }
    else
    {
      std::stringstream str;
      str <<  "Unknown time source specified \"" << time_source << "\".";
      throw kwiver::vital::video_config_exception( str.str() );
    }

    // If we have located a start time in the video, save the PTS for
    // that point in the video. The video should be left positioned
    // where the time was located. We will get the PTS of a frame and,
    // using this pts_of_meta_ts, be able to adjust the time we got
    // from the metadata correctly.
    if (meta_ts != 0)
    {
      pts_of_meta_ts = d_video_stream.current_pts();
      d_frame_time = meta_ts;
    }

    // Tried seeking to the beginning but some videos don't
    // want to seek, even to the start.  So reload the video.
    {
      std::lock_guard< std::mutex > lock( s_open_mutex );
      d_video_stream.open( this->video_path ); // Calls close on current video
    }

    if ( ! d_video_stream.advance() )
    {
      retval = false;
    }

    // if, after advancing, the PTS is still zero, then we can not
    // establish a relative time reference
    d_have_frame_time = ( 0 != d_video_stream.current_pts() );

    // Clear any old metadata
    metadata_collection.clear();

    return retval;
  } // init_timestamp


// ------------------------------------------------------------------
  bool misp_time()
  {
    int frame_count( c_time_scan_frame_limit );
    bool retval(false);
    int64_t ts = 0;

    do
    {
      std::vector< unsigned char > pkt_data = d_video_stream.current_packet_data();

      if ( kwiver::vital::find_MISP_microsec_time(  pkt_data, ts ) )
      {
        meta_ts = ts; // in usec
        LOG_DEBUG( this->d_logger, "Found MISP frame time:" << meta_ts );

        d_have_abs_frame_time = true;
        retval = true;
      }
    }
    while ( ( meta_ts == 0.0 )
            && d_video_stream.advance()
            && (( c_time_scan_frame_limit == 0) || frame_count-- ));

    return retval;
  } // misp_time


// ------------------------------------------------------------------
  bool klv_time( std::string type )
  {
    int frame_count( c_time_scan_frame_limit );
    bool retval(false);

    do
    {
      // skip ahead until we get some metadata
      if ( d_video_stream.current_metadata().empty() )
      {
        continue;
      }

      //It might be more accurate to get the second unique timestamp instead of the first
      std::deque< vxl_byte > curr_md = d_video_stream.current_metadata();
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
                // Get unix timestamp as usec
                meta_ts = meta->find( kwiver::vital::VITAL_META_UNIX_TIMESTAMP ).as_uint64();

                LOG_DEBUG( this->d_logger, "Found initial " << type << " timestamp: " << meta_ts );

                d_have_abs_frame_time = true;
                retval = true;
              } // has time element
            } // correct metadata type
          } // has metadata origin
        } // foreach over all metadata packets
      } // end if processed metadata collection
    }
    while ( ( meta_ts == 0 )
            && d_video_stream.advance()
            && ( (c_time_scan_frame_limit == 0) || frame_count-- ) );

    return retval;
  } // klv_time

}; // end of internal class.

// static open interlocking mutex
std::mutex vidl_ffmpeg_video_input::priv::s_open_mutex;


// ==================================================================
vidl_ffmpeg_video_input
::vidl_ffmpeg_video_input()
  : d( new priv() )
{
  attach_logger( "video_input" ); // get appropriate logger
  d->d_logger = this->m_logger;
}


/// copy constructor
vidl_ffmpeg_video_input
::vidl_ffmpeg_video_input( vidl_ffmpeg_video_input const& other )
  : d( new priv() )
{
  attach_logger( "video_input" ); // get appropriate logger
  d->d_logger = this->m_logger;

  // Copy configuration values only
  d->c_time_scan_frame_limit   = other.d->c_time_scan_frame_limit;
  d->c_start_at_frame          = other.d->c_start_at_frame;
  d->c_stop_after_frame        = other.d->c_stop_after_frame;
}


vidl_ffmpeg_video_input
::~vidl_ffmpeg_video_input()
{
  d->d_video_stream.close( );
}


// ------------------------------------------------------------------
// Get this algorithm's \link vital::config_block configuration block \endlink
vital::config_block_sptr
vidl_ffmpeg_video_input
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config = vital::algo::video_input::get_configuration();

  config->set_value( "time_scan_frame_limit", d->c_time_scan_frame_limit,
                     "Number of frames to be scanned searching input video for embedded time. "
                      "If the value is zero, the whole video will be scanned." );

  config->set_value( "start_at_frame", d->c_start_at_frame,
                     "Frame number (from 1) to start processing video input. "
                     "If set to zero, start at the beginning of the video." );

  config->set_value( "stop_after_frame", d->c_stop_after_frame,
                     "Number of frames to supply. If set to zero then supply all frames after start frame." );

  config->set_value( "absolute_time_source", d->c_time_source,
                     "List of sources for absolute frame time information. "
                     "This entry specifies a comma separated list of sources that are "
                     "tried in order until a valid time source is found. "
                     "If an absolute time source is found, it is used in the output time stamp. "
                     "Absolute times are derived from the metadata in the video stream. "
                     "Valid source names are \"none\", \"misp\", \"klv0601\", \"klv0104\".\n"
                     "Where:\n"
                     "    none - do not supply absolute time\n"
                     "    misp - use frame embedded time stamps.\n"
                     "    klv0601 - use klv 0601 format metadata for frame time\n"
                     "    klv0104 - use klv 0104 format metadata for frame time\n"
                     "Note that when \"none\" is found in the list no further time sources will be evaluated, "
                     "the output timestamp will be marked as invalid, and the HAS_ABSOLUTE_FRAME_TIME capability "
                     "will be set to false.  The same behavior occurs when all specified sources are tried and "
                     "no valid time source is found."
    );

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

  d->c_start_at_frame = config->get_value<vital::timestamp::frame_t>(
    "start_at_frame", d->c_start_at_frame );

  d->c_stop_after_frame = config->get_value<vital::timestamp::frame_t>(
    "stop_after_frame", d->c_stop_after_frame );

  kwiver::vital::tokenize( config->get_value<std::string>( "time_source", d->c_time_source ),
            d->c_time_source_list, " ,", true );
}


// ------------------------------------------------------------------
bool
vidl_ffmpeg_video_input
::check_configuration(vital::config_block_sptr config) const
{
  bool retcode(true); // assume success

  // validate time source
  bool valid_src( true );
  std::vector< std::string > time_source;
  kwiver::vital::tokenize( config->get_value<std::string>( "time_source", d->c_time_source ),
            time_source, " ,", true );

  VITAL_FOREACH( auto source, time_source )
  {
    if (source != "none"
        && source != "misp"
        && source != "klv0601"
        && source != "klv0104")
    {
      valid_src = false;
      break;
    }
  }

  if ( ! valid_src )
  {
    LOG_ERROR( m_logger, "time source must be a comma separated list of one or more "
               "of the following strings: \"none\", \"misp\", \"klv0601\", \"klv0104\"" );
    retcode = false;
  }

  // validate start frame
  vital::timestamp::frame_t frame =  config->get_value<vital::timestamp::frame_t>( "start_at_frame");
  //  zero indicates not set, otherwise must be 1 or greater
  if (frame < 0 )
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
  this->close(); // close video stream and reset internal state

  d->video_path = video_name;

  // If the open succeeds, it will already have read the first frame.
  // avcodec_open2 which is called by open is not thread safe so we need to lock.
  {
    std::lock_guard< std::mutex > lock( d->s_open_mutex );
    if( ! d->d_video_stream.open( video_name ) )
    {
      // Throw exception
      throw kwiver::vital::file_not_found_exception( video_name, "Open failed" );
    }
  }

  d->d_at_eov = false;
  d->d_frame_number = 1;

  if ( d->c_start_at_frame != 0 &&  d->c_start_at_frame > 1 )
  {
    // move stream to specified frame number
    unsigned int frame_num = d->d_video_stream.frame_number(); // get first/current frame number

    while (frame_num < d->c_start_at_frame)
    {
      if( ! d->d_video_stream.advance() )
      {
        break;
      }

      frame_num = d->d_video_stream.frame_number();
    }
  }

  // check for metadata
  d->d_have_metadata = d->d_video_stream.has_metadata();

  // We already have required frame
  // See if we can generate a time base
  d->d_have_frame = true;
  bool time_found( false );
  VITAL_FOREACH( auto time_source, d->c_time_source_list )
  {
    LOG_DEBUG( d->d_logger, "Looking for " << time_source << " as time source" );
    if( d->init_timestamp( time_source ) )  // will call advance()
    {
      LOG_DEBUG( d->d_logger, "Found " << time_source << " as time source" );
      time_found = true;
      break;
    }
  }

  if ( ! time_found )
  {
    LOG_ERROR( m_logger, "Failed to initialize the timestamp for: " << d->video_path );
    throw kwiver::vital::video_stream_exception( "could not initialize timestamp" );
  }

  // Set capabilities
  set_capability(vital::algo::video_input::HAS_TIMEOUT, false );

  set_capability(vital::algo::video_input::HAS_EOV, true );
  set_capability(vital::algo::video_input::HAS_FRAME_NUMBERS, true );
  set_capability(vital::algo::video_input::HAS_FRAME_TIME, d->d_have_frame_time  );
  set_capability(vital::algo::video_input::HAS_ABSOLUTE_FRAME_TIME,
                 (d->d_have_frame_time & d->d_have_abs_frame_time) );
  set_capability(vital::algo::video_input::HAS_METADATA, d->d_have_metadata  );
}


// ------------------------------------------------------------------
void
vidl_ffmpeg_video_input
::close()
{
  d->d_video_stream.close();

  d->d_have_frame = false;
  d->d_at_eov = false;
  d->d_have_frame_time = false;
  d->d_have_abs_frame_time = false;
  d->d_have_metadata = false;
  d->d_frame_time = 0;
  d->d_frame_number = 1;
}


// ------------------------------------------------------------------
bool
vidl_ffmpeg_video_input
::next_frame( kwiver::vital::timestamp& ts,
              uint32_t timeout )
{
  if (d->d_at_eov)
  {
    return false;
  }

  // is stream open?
  if ( ! d->d_video_stream.is_open() )
  {
    throw vital::file_not_read_exception( d->video_path, "Video not open" );
  }

  // Sometimes we already have the frame available.
  if ( d->d_have_frame )
  {
    d->d_have_frame = false;
  }
  else
  {
    if( ! d->d_video_stream.advance() )
    {
      d->d_at_eov = true;
      return false;
    }
  }

  unsigned int frame_num = d->d_video_stream.frame_number();
  if( (d->c_stop_after_frame != 0) && (d->c_stop_after_frame < frame_num ))
  {
    d->d_at_eov = true;  // logical end of file
    return false;
  }

  // ---- Calculate time stamp ----
  // Metadata packets may not exist for each frame, so use the diff in
  // presentation time stamps to foward the first metadata time stamp.
  double pts_diff = ( d->d_video_stream.current_pts() - d->pts_of_meta_ts ) * 1e6;
  d->d_frame_time = d->meta_ts + pts_diff;
  d->d_frame_number = d->d_video_stream.frame_number();

  // We don't always have all components of a timestamp, so start with
  // an invalid TS and add the data we have.
  ts.set_invalid();
  ts.set_frame( d->d_frame_number );

  if ( d->d_have_frame_time )
  {
    ts.set_time_usec( d->d_frame_time );
  }

  // ---- process metadata ---
  d->metadata_collection.clear(); // erase old metadata packets

  return true;
}


// ------------------------------------------------------------------
kwiver::vital::image_container_sptr
vidl_ffmpeg_video_input
::frame_image( )
{
  if (d->d_at_eov)
  {
    return kwiver::vital::image_container_sptr();
  }

  // We succeed in the step if we can convert the frame to RGB.
  vil_image_view<vxl_byte> img;
  vidl_frame_sptr vidl_frame = d->d_video_stream.current_frame();
  bool result = vidl_convert_to_view( *vidl_frame,
                                      img,
                                      VIDL_PIXEL_COLOR_RGB );

  if ( ! result )
  {
    throw kwiver::vital::video_stream_exception( "could not convert image to vidl format" );
  }

  return vital::image_container_sptr( new vxl::image_container( img ) );
}


// ------------------------------------------------------------------
kwiver::vital::video_metadata_vector
vidl_ffmpeg_video_input
::frame_metadata()
{
  if (d->d_at_eov)
  {
    return kwiver::vital::video_metadata_vector();
  }

  // ---- process metadata ---
  // If the vector is empty, then try to convert metadata.
  if ( d->metadata_collection.empty() )
  {
    std::deque< uint8_t > curr_md = d->d_video_stream.current_metadata();
    if (curr_md.size() > 0 )
    {
      // will manage metadata collection object.
      d->process_metadata( curr_md );
    }
  }

  return d->metadata_collection;
}


// ------------------------------------------------------------------
bool
vidl_ffmpeg_video_input
::end_of_video() const
{
  return d->d_at_eov;
}


// ------------------------------------------------------------------
bool
vidl_ffmpeg_video_input
::good() const
{
  return d->d_video_stream.is_valid();
}

} } } // end namespace
