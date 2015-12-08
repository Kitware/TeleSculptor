## Author = 'jonathan.owens'

require_relative './krtd.rb'

def load_camera(file_path)
  model = Sketchup.active_model
  ents = model.active_entities
  pages = Sketchup.active_model.pages
  view = Sketchup.active_model.active_view
  
  krtd_ff = from_file(file_path)
  
  cam_point3d = Geom::Point3d.new( krtd_ff.camera[0], krtd_ff.camera[1], krtd_ff.camera[2] )
  target_point3d = Geom::Point3d.new( krtd_ff.target[0], krtd_ff.target[1], krtd_ff.target[2] )
  up_point3d = Geom::Vector3d.new( krtd_ff.up[0], krtd_ff.up[1], krtd_ff.up[2] )
  
  new_cam = Sketchup::Camera.new( cam_point3d, target_point3d, up_point3d )
  new_cam.fov = krtd_ff.fov_y
  new_cam.aspect_ratio = krtd_ff.x_dim / krtd_ff.y_dim
  view.camera = new_cam
  
  return new_cam, cam_point3d
end


