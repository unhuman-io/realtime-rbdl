
set(ENV{RBDL_PATH} ${PROJECT_SOURCE_DIR}/rbdl/build/artifacts)
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/rbdl/examples/urdfreader )
FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)

add_executable(tmp tmp.cpp)
target_include_directories(tmp PUBLIC ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(tmp
	${RBDL_LIBRARY}
	${RBDL_URDFReader_LIBRARY}
    yaml-cpp
)
