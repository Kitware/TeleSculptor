## Author = 'jonathan.owens'

require 'matrix'

# Because matrices are immutable in Ruby, apparantly, so this allows us to 
# change elements at a given index. See
# compgroups.net/comp.lang.ruby/matrix-class-how-to-set-a-single-element/769573
# for more information.
class Matrix 
  def []=(i, j, x) 
    @rows[i][j] = x 
  end 
end


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
    @x_dim = 2 * @focal_length_mat[0, 2]
    @y_dim = 2 * @focal_length_mat[1, 2]

    # in pixels
    @focal_length_x = @focal_length_mat[0, 0]
    @focal_length_y = @focal_length_mat[1, 1]

    # initializing the properties that need computing. we'll actually
    # compute them once the getter method is called so as not to waste
    # unneeded computation at initialization.
    @_fov_x = nil
    @_fov_y = nil
    @_up = nil
    @_target = nil
    @_camera = nil
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
    # May not be right...Currently setting it as the second column in the
    # rotation matrix but it may be the second row and may be negated.
    if @_up == nil then
      @_up = -1 * Vector[@rotation_mat[1, 0], 
		    @rotation_mat[1, 1], 
		    @rotation_mat[1, 2]]
    end
    return @_up
  end

  def target
    if @_target == nil then
      @_target = Vector[0, 0, 0]
      @_target = camera + Vector[@rotation_mat[2, 0],
                                 @rotation_mat[2, 1],
                                 @rotation_mat[2, 2]]
    end
    return @_target
  end
  def camera
    if @_camera == nil then
      @_camera = -1 * @rotation_mat.transpose * @translation_vec
    end
    return @_camera
  end

  def eye
    puts "To be implemented"
  end
end

def from_file(fp)
  # Given the filepath of a krtd file, read in the values and create and 
  # return a KRTD object from those values. 
  #   The first three lines are a 3x3 matrix containing focal length 
  # information, where the first two diagonal elements are respectively the x 
  # and y focal lengths, in pixels, and the first two entries in the third 
  # column contains the dimensions (divided by 2) of the image.
  # 	The next three lines (after a blank line) contain the 3x3 rotational
  # matrix.
  #  	The next line (after another blank line) contains the 3x1 translation 
  # vector.
  #   The final line (after again another blank line) contains distortion 
  # information, which at this point we are reading in as a single floating
  # point value. We're not currently doing anything with this information.
  #   See the following lines for an example of a krtd file.
  # 
  # 	12657.4096168       0     		  360
  #   0 				12657.4096168     240
  #   0       			0       		1
  #
  # 	0.869290719549 -0.490764202507 0.0590266248765
  # 	-0.34427466669 -0.686805939408 -0.640134794765
  # 	0.354695078598 0.536141864744 -0.7659917115
  #
  #   0.521841450099 -0.176752729198 51.0380531174
  #
  # 	0
  #
  krtd_file = File.new(fp, "r")
  name = fp
  focal_length_mat = Matrix.zero(3)
  rotation_mat = Matrix.zero(3)
  translation_vec = nil
  distortion = 0
  
  idx = 0 # For line counting
  while (line = krtd_file.gets)
    raw_line = line.strip.split
    if idx < 3 then
      focal_length_mat[idx, 0] = raw_line[0].to_f
      focal_length_mat[idx, 1] = raw_line[1].to_f
      focal_length_mat[idx, 2] = raw_line[2].to_f
    elsif idx > 3 && idx < 7 then
      rotation_mat[idx - 4, 0] = raw_line[0].to_f
      rotation_mat[idx - 4, 1] = raw_line[1].to_f
      rotation_mat[idx - 4, 2] = raw_line[2].to_f
    elsif idx > 7 && idx < 9 then
      translation_vec = Vector[raw_line[0].to_f, raw_line[1].to_f, raw_line[2].to_f]
    elsif idx == 10 then
      distortion = raw_line[0].to_f
    end
    idx += 1
  end
  return KRTD.new(focal_length_mat, rotation_mat, translation_vec, fp, distortion)
end

