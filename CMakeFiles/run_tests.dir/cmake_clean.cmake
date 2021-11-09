file(REMOVE_RECURSE
  "srv_gen"
  "srv_gen"
  "src/graph_navigation/srv"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/run_tests.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
