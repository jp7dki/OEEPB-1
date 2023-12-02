# Find all source files in a single current directory
# Save the name to DIR_examples_SRCS
aux_source_directory(. DIR_fatfs_SRCS)

# Generate the link library
add_library(fatfs ${DIR_fatfs_SRCS})
target_link_libraries(fatfs PUBLIC pico_stdlib hardware_spi hardware_flash)