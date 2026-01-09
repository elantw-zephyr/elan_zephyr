zephyr_include_directories(include)
zephyr_include_directories(soc/elan/em32f967)

# Add driver include directories
add_subdirectory(drivers)
zephyr_library_sources(soc/elan/em32f967/soc_tmp.c)

# Mark it as allowed to be empty
set_target_properties(drivers__clock_control PROPERTIES ALLOW_EMPTY TRUE)
set_target_properties(drivers__gpio PROPERTIES ALLOW_EMPTY TRUE)
set_target_properties(drivers__flash PROPERTIES ALLOW_EMPTY TRUE)
set_target_properties(drivers__serial PROPERTIES ALLOW_EMPTY TRUE)
set_target_properties(drivers__spi PROPERTIES ALLOW_EMPTY TRUE)
#set_target_properties(drivers__entropy PROPERTIES ALLOW_EMPTY TRUE)
#set_target_properties(drivers__bbram PROPERTIES ALLOW_EMPTY TRUE)
