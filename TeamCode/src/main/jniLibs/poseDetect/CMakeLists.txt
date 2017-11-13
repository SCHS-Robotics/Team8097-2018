cmake_minimum_required(VERSION 3.4.1)

add_library( opencv-pose-csvreader
             SHARED
             IMPORTED )
set_target_properties( opencv-pose-csvreader
             PROPERTIES IMPORTED_LOCATION
             libopencv-pose-csvreader.so

add_library( opencv-pose-mesh
             SHARED
             IMPORTED )
set_target_properties( opencv-pose-mesh
             PROPERTIES IMPORTED_LOCATION
             libopencv-pose-mesh.so

add_library( opencv-pose-model
             SHARED
             IMPORTED )
set_target_properties( opencv-pose-model
             PROPERTIES IMPORTED_LOCATION
             libopencv-pose-model.so

add_library( opencv-pose-modelregistration
             SHARED
             IMPORTED )
set_target_properties( opencv-pose-modelregistration
             PROPERTIES IMPORTED_LOCATION
             libopencv-pose-modelregistration.so

add_library( opencv-pose-pnpproblem
             SHARED
             IMPORTED )
set_target_properties( opencv-pose-pnpproblem
             PROPERTIES IMPORTED_LOCATION
             libopencv-pose-pnpproblem.so

add_library( opencv-pose-robustmatcher
             SHARED
             IMPORTED )
set_target_properties( opencv-pose-robustmatcher
             PROPERTIES IMPORTED_LOCATION
             libopencv-pose-robustmatcher.so

add_library( opencv-pose-utils
             SHARED
             IMPORTED )
set_target_properties( opencv-pose-utils
             PROPERTIES IMPORTED_LOCATION
             libopencv-pose-utils.so
