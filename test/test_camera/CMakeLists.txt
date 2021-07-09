# Add test_camera.cpp to executable
add_executable(wv-test-camera "test_camera.cpp")

# Target Include directories
include_directories(wv-test-camera EXTRA_INCLUDES)

# Add shared library link to target
target_link_libraries(wv-test-camera
    PUBLIC
    fmt::fmt-header-only

    PRIVATE
    wv-mv-video-capture
)

add_test(NAME test-wv-test-camera COMMAND COMMAND wv-test-camera)
