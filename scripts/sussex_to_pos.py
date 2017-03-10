#!/usr/bin/env python
#ckwg +28
# Copyright 2015 by Kitware, Inc. All Rights Reserved. Please refer to
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  * Neither name of Kitware, Inc. nor the names of any contributors may be used
#    to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
This script is used to read Sussex MUTC metadata and produce POS metadata.
"""

from __future__ import division, print_function
from optparse import OptionParser
import glob, os
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d


def convert_pos2krtd_dir(frame_pattern, sussex_dir, pos_dir):
    """
    
    The POS data format stores each frame's worth of metadata in its own text 
    file with the same name as the frame. The data is stored in the following 
    order:
    
    Element   Data Type           Description
       0       double      Sensor yaw angle (degrees)
       1       double      Sensor pitch angle (degrees)
       2       double      Sensor roll angle (degrees)
       3       double      Sensor latitude in (decimal degrees)
       4       double      Sensor longitude in (decimal degrees)
       5       double      Sensor altitude (feet)
       6       double      GPS time - seconds in week
       7        int        GPS time - week of year
       8       double      Velocity in the North direction (m/s)
       9       double      Velocity in the East direction (m/s)
       10      double      Velocity in the up direction (m/s)
       11       int        IMU status
       12       int        Local adjustment
       13       int        Flags ?
      
    Arguements:
    frame_dir : str
        Global pattern for image files.
    sussex_dir : str
        Directory for the Sussex MUTC metadata csv.
    
    Example
    frame_pattern = '/media/sf_Matt/data/24_Sept_2015_WAMI_Flight_1/frames/*.jpg'
    sussex_dir = '/media/sf_Matt/data/24_Sept_2015_WAMI_Flight_1/novatel_ins_data_2015-09-24-10-01-18'
    pos_dir = '/media/sf_Matt/data/24_Sept_2015_WAMI_Flight_1/pos'
    """
    
    # Frind frames
    frame_fnames = glob.glob(frame_pattern)
    
    # Extract the frame number
    frame_index = np.zeros(len(frame_fnames), dtype=np.int)
    for i in range(len(frame_fnames)):
        fnamei = os.path.split(frame_fnames[i])[1]
        frame_index[i] = int(fnamei[fnamei.find('frame')+5:fnamei.find('_')])
    
    
    # -------------------- Parse RawImageTopic.csv ---------------------------
    fname = glob.glob(''.join([sussex_dir,'/*RawImageTopic.csv']))[0]
    raw_image_topic_df = pd.read_csv(fname)
    column_names = ['time','GPSWeek','GPSTime','FrameNumber','ImageHeight',
                    'ImageWidth','BitDepth','BufferNumber']
    raw_image_topic_df.columns = column_names
    
    image_height = np.unique(raw_image_topic_df.ImageHeight.as_matrix())
    assert len(image_height) == 1, 'ImageHeight must be unique.'
    image_height = image_height[0]
    
    image_width = np.unique(raw_image_topic_df.ImageWidth.as_matrix())
    assert len(image_width) == 1, 'ImageWidth must be unique.'
    image_width = image_width[0]
    
    bit_depth = np.unique(raw_image_topic_df.BitDepth.as_matrix())
    assert len(bit_depth) == 1, 'BitDepth must be unique.'
    bit_depth = bit_depth[0]
    
    # Define mapping from frame number to UTC time
    frame_times = dict(zip(raw_image_topic_df.FrameNumber.tolist(), 
                           raw_image_topic_df.time.tolist()))
    
    # Define mapping from frame number to gps_week
    gps_week = dict(zip(raw_image_topic_df.FrameNumber.tolist(), 
                        raw_image_topic_df.GPSWeek.tolist()))
    
    # Define mapping from frame number to gps_sec
    gps_sec = dict(zip(raw_image_topic_df.FrameNumber.tolist(), 
                       raw_image_topic_df.GPSTime.tolist()))
    # ------------------------------------------------------------------------
    
    
    # --------------------------- Parse pva.csv ------------------------------
    fname = glob.glob(''.join([sussex_dir,'/*pva.csv']))[0]
    pva_df = pd.read_csv(fname)
    pva_df.columns = ['time','seq','stamp','frame_id','lat','lon','height',
                      'vel_n','vel_e','vel_u','roll','pitch','azimuth']
        
    # Create interpolation objects against time.
    time        = pva_df.time.as_matrix()
    
    # Remove large offset so that the interpolation is better behaved.
    t0 = time[0]
    time = time - t0
    
    get_lat     = interp1d(time, pva_df.lat.tolist())
    get_lon     = interp1d(time, pva_df.lon.tolist())
    get_height  = interp1d(time, pva_df.height.tolist())
    get_roll    = interp1d(time, pva_df.roll.tolist())
    get_pitch   = interp1d(time, pva_df.pitch.tolist())
    get_azimuth = interp1d(time, pva_df.azimuth.tolist())
    # ------------------------------------------------------------------------
    
    
    # --------------------------- Parse pos_1.csv ------------------------------
    fname = glob.glob(''.join([sussex_dir,'/*pos_1.csv']))[0]
    pos_1_df = pd.read_csv(fname)
    pos_1_df.columns = ['time','seq','stamp','frame_id','status','service',
                        'latitude','longitude','altitude',
                        'position_covariance0','position_covariance1',
                        'position_covariance2','position_covariance3',
                        'position_covariance4','position_covariance5',
                        'position_covariance6','position_covariance7',
                        'position_covariance8','position_covariance_type']
    
    # Create interpolation objects against time.
    time        = pos_1_df.time.tolist()
    
    if  False:
        # Check that the values in pos_1.csv are consistent with pva.csv.
        
        # Remove large offset so that the interpolation is better behaved.
        t0 = time[0]
        time = time - t0
    
        print('Maximum difference between pos_1.csv and pva.csv latitude',
              np.max(np.abs(get_lat(time) - pos_1_df.latitude.tolist())))
        print('Maximum difference between pos_1.csv and pva.csv longitude',
              np.max(np.abs(get_lon(time) - pos_1_df.longitude.tolist())))
        print('Maximum difference between pos_1.csv and pva.csv altitude',
              np.max(np.abs(get_height(time) - pos_1_df.altitude.tolist())))
    # ------------------------------------------------------------------------
    
    
    try:
        os.mkdir(pos_dir)
    except:
        pass
    
    # Write out one POS txt file per image frame.
    for i in range(len(frame_fnames)):
        # The POS file name will match that of the image frame.
        img_fname = os.path.splitext(os.path.split(frame_fnames[i])[1])[0]
        pos_fname = ''.join([pos_dir,'/',img_fname,'.txt'])
        
        pos_datai = np.zeros(14)
        
        # Frame time in UTC nanoseconds.
        timei = frame_times[frame_index[i]]
        
        # ---------------- Determine Orientation of Camera -------------------
        ins_yaw = get_azimuth(timei-t0)     # (degrees)
        ins_pitch = get_pitch(timei-t0)     # (degrees)
        ins_roll = get_roll(timei-t0)       # (degrees)
        # --------------------------------------------------------------------
                
        # Sensor latitude in (decimal degrees)
        pos_datai[3] = get_lat(timei-t0)
        
        # Sensor longitude in (decimal degrees)
        pos_datai[4] = get_lon(timei-t0)
        
        # Sensor altitude (feet)
        pos_datai[5] = get_height(timei-t0)
        
        # GPS time - seconds in week
        pos_datai[6] = gps_sec[frame_index[i]]
        
        # Validate against the image filename
        err = int(img_fname.split('_')[1]) - gps_sec[frame_index[i]]
        assert np.abs(err) < 1, 'GPS seconds in image filename does not match.'
        
        # GPS time - week of year
        pos_datai[7] = gps_week[frame_index[i]]
        
        # Velocity in the North direction (m/s)
        pos_datai[8] = 0
        
        # Velocity in the East direction (m/s)
        pos_datai[9] = 0
        
        # Velocity in the up direction (m/s)
        pos_datai[10] = 0
        
        # IMU status
        pos_datai[11] = 3   # NOT SURE WHAT THIS DOES
        
        # Local adjustment
        pos_datai[12] = 4   # NOT SURE WHAT THIS DOES
        
        # Flags ?
        pos_datai[13] = 1   # NOT SURE WHAT THIS DOES
        
        np.savetxt(pos_fname, np.atleast_2d(pos_datai), delimiter=',',
                   fmt='%.10f,'*7 + '%i,' + '%.10f,'*3 + '%i,'*2 + '%i')


def main():
    usage = "usage: %prog [options] frame_pattern sussex_dir pos_dir"
    description = "Convert Sussex MUTC metadata to POS metadata"
    parser = OptionParser(usage=usage, description=description)

    (options, args) = parser.parse_args()
    
    if len(args) < 3:
        parser.print_help()
        return
    
    frame_pattern = args[0]
    sussex_dir = args[1]
    pos_dir = args[2]

    convert_pos2krtd_dir(frame_pattern, sussex_dir, pos_dir)


if __name__ == "__main__":
    main()
