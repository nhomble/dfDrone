FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/dfDrone/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/DFDMessage.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_DFDMessage.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
