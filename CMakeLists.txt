cmake_minimum_required(VERSION 3.21)
project(ApexSim VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(APEX_BUILD_TESTS "Build the headless physics test runner" ON)
option(APEX_NATIVE_ARCH "Optimise for the build machine's CPU (AVX2 etc.)" OFF)

include(FetchContent)
set(FETCHCONTENT_QUIET ON)

# ---------------------------------------------------------------- GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
if(UNIX AND NOT APPLE)
  set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
endif()
FetchContent_Declare(glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
  GIT_SHALLOW TRUE)

# ---------------------------------------------------------------- GLM
FetchContent_Declare(glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 1.0.1
  GIT_SHALLOW TRUE)

# ---------------------------------------------------------------- Dear ImGui (no CMake upstream)
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.9b
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR _no_cmake_)

# ---------------------------------------------------------------- miniaudio (single header)
FetchContent_Declare(miniaudio
  GIT_REPOSITORY https://github.com/mackron/miniaudio.git
  GIT_TAG 0.11.22
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR _no_cmake_)

FetchContent_MakeAvailable(glfw glm imgui miniaudio)

add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC glfw)

# ---------------------------------------------------------------- vendored: glad + stb
add_library(apex_thirdparty STATIC
  third_party/glad/src/gl.c
  third_party/thirdparty_impl.cpp)
target_include_directories(apex_thirdparty PUBLIC
  third_party/glad/include
  third_party/stb
  ${miniaudio_SOURCE_DIR})
find_package(Threads REQUIRED)
target_link_libraries(apex_thirdparty PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
if(UNIX AND NOT APPLE)
  target_link_libraries(apex_thirdparty PUBLIC m)
endif()

# ---------------------------------------------------------------- common compile settings
function(apex_target_settings tgt)
  if(MSVC)
    target_compile_options(${tgt} PRIVATE /W3 /permissive- /MP /utf-8 /Zc:__cplusplus)
    target_compile_definitions(${tgt} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
    if(APEX_NATIVE_ARCH)
      target_compile_options(${tgt} PRIVATE /arch:AVX2)
    endif()
  else()
    target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
    if(APEX_NATIVE_ARCH)
      target_compile_options(${tgt} PRIVATE -march=native)
    endif()
  endif()
  target_compile_definitions(${tgt} PRIVATE GLM_FORCE_XYZW_ONLY GLM_ENABLE_EXPERIMENTAL)
endfunction()

# ---------------------------------------------------------------- simulation core (no GL / window)
# Physics, track queries and audio synthesis live here so the test runner can use them headless.
add_library(apex_sim STATIC
  src/core/Log.cpp
  src/core/Paths.cpp
  src/core/Settings.cpp
  src/core/Noise.cpp
  src/world/TrackData.cpp
  src/car/Tire.cpp
  src/car/Drivetrain.cpp
  src/car/Car.cpp
  src/car/CarParams.cpp
  src/game/AIDriver.cpp
  src/game/LapTimer.cpp
  src/audio/EngineSynth.cpp
  src/audio/CarAudio.cpp)
target_include_directories(apex_sim PUBLIC src)
target_link_libraries(apex_sim PUBLIC glm::glm apex_thirdparty)
apex_target_settings(apex_sim)

# ---------------------------------------------------------------- game executable
add_executable(apexsim
  src/main.cpp
  src/core/Input.cpp
  src/gfx/GLUtil.cpp
  src/gfx/Shader.cpp
  src/gfx/Texture.cpp
  src/gfx/Mesh.cpp
  src/gfx/Framebuffer.cpp
  src/gfx/Sky.cpp
  src/gfx/Shadows.cpp
  src/gfx/PostFX.cpp
  src/gfx/Renderer.cpp
  src/gfx/ProcTextures.cpp
  src/world/TrackMesh.cpp
  src/world/Terrain.cpp
  src/world/Vegetation.cpp
  src/car/CarModel.cpp
  src/audio/AudioDevice.cpp
  src/game/App.cpp
  src/game/Session.cpp
  src/game/CameraRig.cpp
  src/ui/UiTheme.cpp
  src/ui/MainMenu.cpp
  src/ui/SettingsMenu.cpp
  src/ui/Hud.cpp)
target_link_libraries(apexsim PRIVATE apex_sim imgui glfw)
apex_target_settings(apexsim)

if(WIN32)
  set_target_properties(apexsim PROPERTIES WIN32_EXECUTABLE $<CONFIG:Release>)
  if(MSVC)
    target_link_options(apexsim PRIVATE $<$<CONFIG:Release>:/ENTRY:mainCRTStartup>)
  endif()
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT apexsim)
  set_target_properties(apexsim PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT _apex_ipo OUTPUT _apex_ipo_msg LANGUAGES CXX)
if(_apex_ipo)
  set_property(TARGET apexsim apex_sim PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif()

# Tell the executable where the source assets live (it also searches next to the exe).
target_compile_definitions(apexsim PRIVATE APEX_SOURCE_ASSETS="${CMAKE_SOURCE_DIR}/assets")
target_compile_definitions(apex_sim PRIVATE APEX_SOURCE_ASSETS="${CMAKE_SOURCE_DIR}/assets")

# Copy assets next to the binary so the build folder can be zipped and run anywhere.
add_custom_command(TARGET apexsim POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:apexsim>/assets
  COMMENT "Copying assets")

# ---------------------------------------------------------------- tests
if(APEX_BUILD_TESTS)
  add_executable(apexsim_tests tests/physics_tests.cpp)
  target_link_libraries(apexsim_tests PRIVATE apex_sim)
  apex_target_settings(apexsim_tests)
  target_compile_definitions(apexsim_tests PRIVATE APEX_SOURCE_ASSETS="${CMAKE_SOURCE_DIR}/assets")
endif()
