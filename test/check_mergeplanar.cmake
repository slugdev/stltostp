# Regression check for the mergeplanar option: converting a unit cube
# (12 triangles) with mergeplanar must produce exactly 6 planar faces,
# 12 edge curves and 8 vertices - i.e. the coplanar triangle pairs were
# merged and the face-diagonal edges removed.
#
# Usage: cmake -DEXE=<stltostp> -DDIR=<test dir> -P check_mergeplanar.cmake

cmake_minimum_required(VERSION 3.12)

execute_process(
	COMMAND "${EXE}" "${DIR}/cube.stl" "${DIR}/cube_merged.stp" mergeplanar
	RESULT_VARIABLE res
)
if(NOT res EQUAL 0)
	message(FATAL_ERROR "stltostp failed with exit code ${res}")
endif()

file(READ "${DIR}/cube_merged.stp" content)

macro(count_entities keyword expected)
	string(REGEX MATCHALL "= ${keyword}\\(" matches "${content}")
	list(LENGTH matches n)
	if(NOT n EQUAL ${expected})
		message(FATAL_ERROR "expected ${expected} ${keyword} entities, found ${n}")
	endif()
endmacro()

count_entities(ADVANCED_FACE 6)
count_entities(EDGE_CURVE 12)
count_entities(VERTEX_POINT 8)

# without mergeplanar the same model must keep one face per triangle
execute_process(
	COMMAND "${EXE}" "${DIR}/cube.stl" "${DIR}/cube_unmerged.stp"
	RESULT_VARIABLE res
)
if(NOT res EQUAL 0)
	message(FATAL_ERROR "stltostp (no mergeplanar) failed with exit code ${res}")
endif()
file(READ "${DIR}/cube_unmerged.stp" content)
count_entities(ADVANCED_FACE 12)
count_entities(EDGE_CURVE 18)
