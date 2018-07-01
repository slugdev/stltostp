PROJECT (stltostp)

SET(STLTOSTP_SRC
StepKernel.cpp
main.cpp)
ENABLE_TESTING()

set (CMAKE_CXX_STANDARD 11)

ADD_EXECUTABLE (stltostp ${STLTOSTP_SRC})

ADD_TEST(
	NAME cat_dish
	COMMAND stltostp cat_dish.stl cat_dish.stp tol .0000001
	WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/test"
)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION "bin/")