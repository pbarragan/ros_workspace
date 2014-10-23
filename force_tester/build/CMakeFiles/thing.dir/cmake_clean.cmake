FILE(REMOVE_RECURSE
  "CMakeFiles/thing.dir/src/thing.o"
  "../bin/thing.pdb"
  "../bin/thing"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/thing.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
