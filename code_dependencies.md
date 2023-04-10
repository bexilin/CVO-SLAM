We modify parts of code in CVO, DVO SLAM and ORB SLAM, and use them in our code. They all use liscence GNU GENERAL PUBLIC LICENSE Version 3. We list the files in our code that are modified version of files from the code of CVO, DVO SLAM, and ORB-SLAM2, and explain our modifications below. 

## Codes from CVO

In src/cvo/include/rkhs_se3.hpp: We 1) add the claim of a new class inn_p; 2) claim new member variables and functions in class rkhs_se3.

In src/cvo/src/rkhs_se3.cpp: We add definition of new functions in class rkhs_se3.

In src/cvo/cpp/pcd_generator.cpp: We add calibration parameters of ETH3D datset to function pcd_generator::get_points_from_pixels.

## Codes from DVO SLAM

In src/cvo_slam/include, keyframe_graph.h, keyframe_tracker.h, local_map.h, local_tracker.h: We 1) remove some original member functions and variables in class KeyframeGraph, KeyframeTracker, LocalMap and LocalTracker; 2) claim new member functions and variables in these classes; 3) remove some original hearder files and add new hearder files; 4) change the namespace from dvo_slam to cvo_slam.

In src/cvo_slam/include/map_serializer.h: We 1) remove some subclasses of MapSerializerInterface; 2) remove some original hearder files and add new hearder files; 3) change the namespace from dvo_slam to cvo_slam.

In src/cvo_slam/src, keyframe_graph.cpp, keyframe_tracker.cpp, local_map.cpp, local_tracker.cpp: We 1) add definition of new functions in class KeyframeGraph, KeyframeTracker, LocalMap and LocalTracker; 2) remove some original member functions and variables in class KeyframeGraphImpl, KeyframeTracker::Impl, LocalMapImpl and LocalTrackerImpl; 3) In classes metioned in 2), claim new member functions and variables, and add function definitions; 4) change the namespace from dvo_slam to cvo_slam.

In src/cvo_slam/include/map_serializer.cpp: We remove definition of some subclasses of MapSerializerInterface.

## Codes from ORB-SLAM2

In src/cvo_slam/thirdparty/ORB_SLAM2/include/Converter.h: We 1) remove some original member functions in class Converter; 2) remove some original hearder files; 3) add new functions Converter::toCvMat and Converter::toAffine.

In src/cvo_slam/thirdparty/ORB_SLAM2/include/ORBextractor.h: We add a new function ORBextractor::ExtractORB.

In src/cvo_slam/thirdparty/ORB_SLAM2/include/ORBmatcher.h: We 1) remove some original member functions in class ORBmatcher; 2) remove some original hearder files; 3) add a new function ORBmatcher::GetMatches; 4) add PCL library header files.

In src/cvo_slam/thirdparty/ORB_SLAM2/src/Converter.cc: We 1) remove definition of some original member functions in class Converter; 2) add definition of functions Converter::toCvMat and Converter::toAffine.

In src/cvo_slam/thirdparty/ORB_SLAM2/src/ORBextractor.cc: We add definition of function ORBextractor::ExtractORB.

In src/cvo_slam/thirdparty/ORB_SLAM2/src/ORBmatcher.cc: We 1) remove definition of some original member functions in class ORBmatcher; 2) add definition of function ORBmatcher::GetMatches.






