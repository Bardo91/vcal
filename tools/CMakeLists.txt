find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
set(CMAKE_AUTOMOC ON)


file(GLOB_RECURSE PIDTUNEGUI_SOURCE_FILES "pid_tune_gui/*.cpp" "pid_tune_gui/*.c" "pid_tune_gui/.h" "pid_tune_gui/.inl")

add_executable(pid_tune_gui ${PIDTUNEGUI_SOURCE_FILES})

target_include_directories(pid_tune_gui PRIVATE ../include)

target_link_libraries(pid_tune_gui LINK_PRIVATE vcal)
target_link_libraries(pid_tune_gui LINK_PRIVATE Qt5::Widgets)
target_link_libraries(pid_tune_gui LINK_PRIVATE Qt5::PrintSupport)