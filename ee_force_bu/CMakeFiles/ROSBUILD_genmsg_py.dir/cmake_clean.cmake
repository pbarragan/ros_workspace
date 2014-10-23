FILE(REMOVE_RECURSE
  "msg_gen"
  "src/ee_force/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/ee_force/msg/__init__.py"
  "src/ee_force/msg/_eeForceMsg.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
