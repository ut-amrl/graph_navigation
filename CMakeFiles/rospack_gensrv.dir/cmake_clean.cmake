file(REMOVE_RECURSE
  "src/graph_navigation/srv"
  "srv_gen"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/rospack_gensrv.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
