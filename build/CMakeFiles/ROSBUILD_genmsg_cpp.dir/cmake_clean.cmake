FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/dfDrone/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/dfDrone/DFDMessage.h"
  "../msg_gen/cpp/include/dfDrone/DFDVelocity.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
