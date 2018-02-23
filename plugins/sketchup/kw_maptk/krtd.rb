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

## Author = 'jonathan.owens'
## Author = 'david.russell'

class KRTD
  def initialize(focal_length_mat,
         rotation_mat,
         translation_vec,
                 name = nil,
         distortion = nil)
    @focal_length_mat = focal_length_mat
    @rotation_mat = rotation_mat
    @translation_vec = translation_vec
    @_name = name

    # dimensions are stored by half in the krtd files, in pixels
    # the *_mat variables are 1D representations of 3x3 matrices
    @x_dim = 2 * @focal_length_mat[0 * 3 + 2]
    @y_dim = 2 * @focal_length_mat[1 * 3 + 2]

    # in pixels
    @focal_length_x = @focal_length_mat[0 * 3 + 0]
    @focal_length_y = @focal_length_mat[1 * 3 + 1]

    # initializing the properties that need computing. we'll actually
    # compute them once the getter method is called so as not to waste
    # unneeded computation at initialization.
    @_fov_x = nil
    @_fov_y = nil
    @_up = nil
    @_target = nil
    @_position = nil
  end

  def name
    return @_name
  end

  def x_dim
    return @x_dim
  end

  def y_dim
    return @y_dim
  end

  def focal_length_x
    return @focal_length_x
  end

  def focal_length_y
    return @focal_length_y
  end
    
  def fov_x
    @_fov_x = @_fov_x ? @_fov_x 
             : 2 * Math.atan((@x_dim / 2.0) / @focal_length_x) * 180 / Math::PI
    return @_fov_x
  end
    
  def fov_y
    @_fov_y = @_fov_y ? @_fov_y
            : 2 * Math.atan((@y_dim / 2.0) / @focal_length_y) * 180 / Math::PI
    return @_fov_y
  end

  def up
    if @_up == nil then
      @_up = [@rotation_mat[3] * -1,
            @rotation_mat[4] * -1,
            @rotation_mat[5] * -1]
    end
    return @_up
  end

  def target
    if @_target == nil then
      position #sets the value of @_position if it wasn't set previously 
      @_target = [@_position[0] + @rotation_mat[6],
                @_position[1] + @rotation_mat[7],
                @_position[2] + @rotation_mat[8]]
    end
    return @_target
  end

  def position
    if @_position == nil then
      @_position = [0,0,0]
      #@_position = -1 * @rotation_mat.transpose * @translation_vec
      @_position[0] = -1 * (@rotation_mat[0] * @translation_vec[0]+
                     @rotation_mat[3] * @translation_vec[1] +
                     @rotation_mat[6] * @translation_vec[2])
      @_position[1] = -1 * (@rotation_mat[1] * @translation_vec[0]+
                     @rotation_mat[4] * @translation_vec[1] +
                     @rotation_mat[7] * @translation_vec[2])
      @_position[2] = -1 * (@rotation_mat[2] * @translation_vec[0]+
                     @rotation_mat[5] * @translation_vec[1] +
                     @rotation_mat[8] * @translation_vec[2])
    end
    return @_position
  end

end

def from_file(fp)
  # Given the filepath of a krtd file, read in the values and create and
  # return a KRTD object from those values.
  #   The first three lines are a 3x3 matrix containing focal length
  # information, where the first two diagonal elements are respectively the x
  # and y focal lengths, in pixels, and the first two entries in the third
  # column contains the principal point which is assumed to be at half of the
  # image width and height.
  #     The next three lines (after a blank line) contain the 3x3 rotational
  # matrix.
  #      The next line (after another blank line) contains the 3x1 translation
  # vector.
  #   The final line (after again another blank line) contains distortion
  # information, which at this point we are reading in as a single floating
  # point value. We're not currently doing anything with this information.
  #   See the following lines for an example of a krtd file.
  #
  #   f_x  0    pp_x
  #   0    f_y  pp_y
  #   0    0    1
  #
  #   R_11 R_12 R_13
  #   R_21 R_22 R_23
  #   R_31 R_32 R_33
  #
  #   t_x t_y t_z
  #
  #   0
  #
  krtd_file = File.new(fp, "r")
  name = fp
  #SketchUp version 8 does not support Ruby Gems. As such, there is no clear way to get
  #a 2D matrix class. The following variables are Ruby Arrays which represent 3x3 matrices.
  focal_length_mat = [0,0,0,0,0,0,0,0,0]
  rotation_mat = [0,0,0,0,0,0,0,0,0]
  translation_vec = nil
  distortion = 0
  idx = 0 # For line counting
  while (line = krtd_file.gets)
    raw_line = line.strip.split
    if idx < 3 then 
      focal_length_mat[idx * 3 + 0] = raw_line[0].to_f
      focal_length_mat[idx * 3 + 1] = raw_line[1].to_f
      focal_length_mat[idx * 3 + 2] = raw_line[2].to_f
    elsif idx > 3 && idx < 7 then 
      rotation_mat[(idx - 4) * 3 + 0] = raw_line[0].to_f
      rotation_mat[(idx - 4) * 3 + 1] = raw_line[1].to_f
      rotation_mat[(idx - 4) * 3 + 2] = raw_line[2].to_f
    elsif idx == 8 then
      translation_vec = [raw_line[0].to_f, raw_line[1].to_f, raw_line[2].to_f]
    elsif idx == 10 then
      distortion = raw_line[0].to_f
    end
    idx += 1
  end
  new_krtd = KRTD.new(focal_length_mat, rotation_mat, translation_vec, fp, distortion)
  return new_krtd
end
