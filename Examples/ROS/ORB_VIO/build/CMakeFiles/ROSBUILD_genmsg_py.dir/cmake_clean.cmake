FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/ORB_VIO/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ORB_VIO/msg/__init__.py"
  "../src/ORB_VIO/msg/_viorb_msg.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
