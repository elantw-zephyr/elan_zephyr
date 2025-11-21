# Add driver include directories
add_subdirectory(drivers)

# Mark it as allowed to be empty
set_target_properties(drivers__clock_control PROPERTIES ALLOW_EMPTY TRUE)
set_target_properties(drivers__gpio PROPERTIES ALLOW_EMPTY TRUE)
set_target_properties(drivers__flash PROPERTIES ALLOW_EMPTY TRUE)
set_target_properties(drivers__serial PROPERTIES ALLOW_EMPTY TRUE)
set_target_properties(drivers__spi PROPERTIES ALLOW_EMPTY TRUE)
#set_target_properties(drivers__entropy PROPERTIES ALLOW_EMPTY TRUE)
#set_target_properties(drivers__bbram PROPERTIES ALLOW_EMPTY TRUE)
