#!/usr/bin/env python
#ckwg +28
# Copyright 2015-2017 by Kitware, Inc. All Rights Reserved. Please refer to
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
This script is used to read Sussex Novatel metadata and produce POS metadata.
"""

from __future__ import division, print_function
from optparse import OptionParser
import glob, os
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from vital.types import Rotation
import matplotlib.pyplot as plt


def quat_from_ypr(yaw, pitch, roll, in_degrees=True):
    """Convert yaw, pitch, and roll (in that order) into a quaternion.
    
    Reference: https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Euler_angles_.28z-y.E2.80.99-x.E2.80.B3_intrinsic.29_.E2.86.92_Quaternion
    
    Arguements:
    yaw : scalar
        Yaw (radians).
    pitch : scalar
        Pitch (radians).
    roll : scalar
        Roll (radians).
    in_degrees : Boolean
        Specifies whether the angles are in degrees.
    
    Return:
    quaternion : 4-array
        Quaternion (w,x,y,z).
    """
    
    if in_degrees:
        yaw = yaw/180*np.pi
        pitch = pitch/180*np.pi
        roll = roll/180*np.pi
    
    cy = np.cos(yaw/2)
    sy = np.sin(yaw/2)
    cp = np.cos(pitch/2)
    sp = np.sin(pitch/2)
    cr = np.cos(roll/2)
    sr = np.sin(roll/2)
    
    quat = np.zeros(4)
    quat[0] = cr*cp*cy + sr*sp*sy
    quat[1] = sr*cp*cy - cr*sp*sy
    quat[2] = cr*sp*cy + sr*cp*sy
    quat[3] = cr*cp*sy - sr*sp*cy
    
    return quat


def ypr_from_quat(quat, in_degrees=True):
    """Convert a quaternion into yaw, pitch, and roll (in that order).
    
    Reference: https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Quaternion_.E2.86.92_Euler_angles_.28z-y.E2.80.99-x.E2.80.B3_intrinsic.29
    Arguements:
    quat : 4-array
        Quaternion (w,x,y,z).
    in_degrees : Boolean
        Specifies whether the output angles should be in degrees.
    
    Return:
    yaw : scalar
        Yaw (radians).
    pitch : scalar
        Pitch (radians).
    roll : scalar
        Roll (radians).
        
    Test:
    quat = np.random.rand(4)*2-1
    quat /= np.linalg.norm(quat)
    yaw, pitch, roll = ypr_from_quat(quat, in_degrees=True)
    quat2 = quat_from_ypr(yaw, pitch, roll, in_degrees=True)
    print(abs(np.dot(quat, quat2)))
    """
    qr,qi,qj,qk = quat
    roll = np.arctan2(2*(qr*qi+qj*qk), 1-2*(qi**2+qj**2))
    pitch = np.arcsin(2*(qr*qj-qk*qi))
    yaw = np.arctan2(2*(qr*qk+qi*qj),1-2*(qj**2+qk**2))
    
    if in_degrees:
        yaw = yaw*180/np.pi
        pitch = pitch*180/np.pi
        roll = roll*180/np.pi
        
    return yaw, pitch, roll


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
    pos_dir : str
        Directory for the output POS files. If it does not currently exist, it 
        will be created.
    
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
    fname = glob.glob(''.join([sussex_dir,'/*mark_1_pva.csv']))[0]
    pva_df = pd.read_csv(fname)
    pva_df.columns = ['time','seq','stamp','frame_id','lat','lon','height',
                      'vel_n','vel_e','vel_u','roll','pitch','azimuth']
        
    # Create interpolation objects against time.
    time        = pva_df.time.as_matrix()
    
    # Remove large offset so that the interpolation is better behaved.
    t0 = time[0]
    time = time - t0
    
    # Latitude (degrees)
    get_lat     = interp1d(time, pva_df.lat.tolist())
    
    # Longitude (degrees)
    get_lon     = interp1d(time, pva_df.lon.tolist())
    
    # Height above WGS84 ellispiod (meters)
    get_height  = interp1d(time, pva_df.height.tolist())
    
    # yaw/pitch/roll in that order (degrees)
    get_roll    = interp1d(time, pva_df.roll.tolist())
    get_pitch   = interp1d(time, pva_df.pitch.tolist())
    get_azimuth = interp1d(time, pva_df.azimuth.tolist())
    
    if True:
        # Show the raw yaw. pitch, and roll.
        plt.subplot('311')
        plt.plot(time, pva_df.azimuth.tolist())
        plt.subplot('312')
        plt.plot(time, pva_df.pitch.tolist())
        plt.subplot('313')
        plt.plot(time, pva_df.roll.tolist())
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
        pos_fname = ''.join([pos_dir,'/',img_fname,'.pos'])
        
        pos_datai = np.zeros(14)
        
        # Frame time in UTC nanoseconds.
        timei = frame_times[frame_index[i]]
        
        # ---------------- Determine Orientation of Camera -------------------
        # yaw/pitch/roll of the INS.
        ins_yaw = get_azimuth(timei-t0)     # (degrees)
        ins_pitch = get_pitch(timei-t0)     # (degrees)
        ins_roll = get_roll(timei-t0)       # (degrees)
        
        # Rotation operator that moves the INS coordinate system from aligned 
        # with the NED axes to its current state.
        ins_rot = Rotation.from_quaternion(quat_from_ypr(ins_yaw, ins_pitch, 
                                                         ins_roll, 
                                                         in_degrees=True))
        
        # Hard-coded camera orientation calibration.  This rotates the INS into
        # the Airforce's camera coordinate system, which is non-standard.
        ins_to_cam_prime = Rotation.from_quaternion(quat_from_ypr(91.625, .867,
                                                              -.612,
                                                              in_degrees=True))
        
        # The AF camera has the look direction as x, the y going out the right,
        # and z going down.  We need to rotate this into x-right, y-down, 
        # z-look direction.
        cp_to_c = Rotation.from_quaternion(quat_from_ypr(90, 0, 90, 
                                                         in_degrees=True))
        
        rot_new = cp_to_c*ins_to_cam_prime*ins_rot
        
        yaw, pitch, roll = ypr_from_quat(rot_new.quaternion(), in_degrees=True)
        
        # Sensor yaw in (degrees)
        pos_datai[0] = yaw
        
        # Sensor pitch in (degrees)
        pos_datai[1] = pitch
        
        # Sensor roll (degrees)
        pos_datai[2] = roll
        # --------------------------------------------------------------------
        
        # Sensor latitude in (decimal degrees)
        pos_datai[3] = get_lat(timei-t0)
        
        # Sensor longitude in (decimal degrees)
        pos_datai[4] = get_lon(timei-t0)
        
        # Sensor altitude (feet above WGS84)
        pos_datai[5] = get_height(timei-t0)*3.28084
        
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
        pos_datai[11] = -1  # NOT SURE WHAT THIS DOES
        
        # Local adjustment
        pos_datai[12] = 0   # NOT SURE WHAT THIS DOES
        
        # Flags ?
        pos_datai[13] = 0   # NOT SURE WHAT THIS DOES
        
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
