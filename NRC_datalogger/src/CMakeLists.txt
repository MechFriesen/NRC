include_directories(../libs)
include_directories(~/Documents/Arduino/libraries)
link_directories(../libs)
link_libraries(arduino_core)

add_executable(NRC_datalogger main.cpp)