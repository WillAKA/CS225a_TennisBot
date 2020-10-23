# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/CS225a_TennisBot)
ADD_EXECUTABLE (simviz_tennis simviz_tennis.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_tennis controller_tennis.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_tennis ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_tennis ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_collision.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY cup.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY tennisBall.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY mmp_panda.urdf DESTINATION ${APP_RESOURCE_DIR})
