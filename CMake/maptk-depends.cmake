# Central location for MAPTK external dependency declaration and resolution

message(STATUS "Entering main DEPENDS file")

include( maptk-depends-Boost )
include( maptk-depends-OpenCV )
include( maptk-depends-PROJ )
include( maptk-depends-VisCL )
include( maptk-depends-VXL )
