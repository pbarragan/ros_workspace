FILE(REMOVE_RECURSE
  "srv_gen"
  "src/slide_box/srv"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/slide_box/srv/__init__.py"
  "src/slide_box/srv/_robot_actuate_object.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
