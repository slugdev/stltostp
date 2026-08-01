# Regression check for https://github.com/slugdev/stltostp issue #6 "KOMPAS.exe
# can not read step result": edge curves are shared between neighboring
# triangles, so the vertices have to be shared as well. Otherwise the edges of
# an EDGE_LOOP reference different VERTEX_POINT entities at the same location
# and readers report a corrupt file structure.
#
# Usage: cmake -DEXE=<stltostp> -DDIR=<test dir> -P check_vertex_sharing.cmake

cmake_minimum_required(VERSION 3.12)

execute_process(
	COMMAND "${EXE}" "${DIR}/cube.stl" "${DIR}/cube_vertex_sharing.stp"
	RESULT_VARIABLE res
)
if(NOT res EQUAL 0)
	message(FATAL_ERROR "stltostp failed with exit code ${res}")
endif()

file(READ "${DIR}/cube_vertex_sharing.stp" content)

# the cube has 8 distinct corners, so one VERTEX_POINT per corner
string(REGEX MATCHALL "= VERTEX_POINT\\(" verts "${content}")
list(LENGTH verts vert_cnt)
if(NOT vert_cnt EQUAL 8)
	message(FATAL_ERROR "expected 8 VERTEX_POINT entities, found ${vert_cnt}")
endif()

# collect the topology: edge curve end vertices, oriented edge direction and
# the oriented edges of every loop
string(REGEX MATCHALL "#[0-9]+ = EDGE_CURVE\\('', #[0-9]+, #[0-9]+," edge_defs "${content}")
foreach(def ${edge_defs})
	string(REGEX MATCH "#([0-9]+) = EDGE_CURVE\\('', #([0-9]+), #([0-9]+)," ignored "${def}")
	set(EDGE_START_${CMAKE_MATCH_1} "${CMAKE_MATCH_2}")
	set(EDGE_END_${CMAKE_MATCH_1} "${CMAKE_MATCH_3}")
endforeach()

string(REGEX MATCHALL "#[0-9]+ = ORIENTED_EDGE\\('',\\*,\\*,#[0-9]+,\\.[TF]\\.\\)" oriented_defs "${content}")
foreach(def ${oriented_defs})
	string(REGEX MATCH "#([0-9]+) = ORIENTED_EDGE\\('',\\*,\\*,#([0-9]+),\\.([TF])\\.\\)" ignored "${def}")
	if(CMAKE_MATCH_3 STREQUAL "T")
		set(ORIENTED_START_${CMAKE_MATCH_1} "${EDGE_START_${CMAKE_MATCH_2}}")
		set(ORIENTED_END_${CMAKE_MATCH_1} "${EDGE_END_${CMAKE_MATCH_2}}")
	else()
		set(ORIENTED_START_${CMAKE_MATCH_1} "${EDGE_END_${CMAKE_MATCH_2}}")
		set(ORIENTED_END_${CMAKE_MATCH_1} "${EDGE_START_${CMAKE_MATCH_2}}")
	endif()
endforeach()

string(REGEX MATCHALL "EDGE_LOOP\\('', \\([^)]*\\)\\)" loop_defs "${content}")
if(loop_defs STREQUAL "")
	message(FATAL_ERROR "no EDGE_LOOP entities found in output")
endif()

# walking a loop must return to its first vertex, using entity ids only
foreach(loop ${loop_defs})
	string(REGEX MATCHALL "#[0-9]+" loop_edges "${loop}")
	list(LENGTH loop_edges edge_cnt)
	math(EXPR last "${edge_cnt} - 1")
	foreach(i RANGE ${last})
		list(GET loop_edges ${i} cur)
		math(EXPR next_i "(${i} + 1) % ${edge_cnt}")
		list(GET loop_edges ${next_i} next)
		string(SUBSTRING "${cur}" 1 -1 cur_id)
		string(SUBSTRING "${next}" 1 -1 next_id)
		if(NOT ORIENTED_END_${cur_id} STREQUAL ORIENTED_START_${next_id})
			message(FATAL_ERROR
				"loop ${loop} is not connected: edge #${cur_id} ends at "
				"#${ORIENTED_END_${cur_id}} but edge #${next_id} starts at "
				"#${ORIENTED_START_${next_id}}")
		endif()
	endforeach()
endforeach()
