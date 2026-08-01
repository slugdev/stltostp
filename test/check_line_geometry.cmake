# Regression check for https://github.com/slugdev/stltostp PR "possible
# incorrect use of surface curve": EDGE_CURVE must reference its LINE
# geometry directly; the schema-invalid two-attribute SURFACE_CURVE entity
# must not be emitted.
#
# Usage: cmake -DEXE=<stltostp> -DDIR=<test dir> -P check_line_geometry.cmake

cmake_minimum_required(VERSION 3.12)

execute_process(
	COMMAND "${EXE}" "${DIR}/single_tri.stl" "${DIR}/single_tri_line.stp"
	RESULT_VARIABLE res
)
if(NOT res EQUAL 0)
	message(FATAL_ERROR "stltostp failed with exit code ${res}")
endif()

file(READ "${DIR}/single_tri_line.stp" content)

if(content MATCHES "SURFACE_CURVE")
	message(FATAL_ERROR "output contains SURFACE_CURVE; EDGE_CURVE must reference LINE directly")
endif()

# every EDGE_CURVE must point at a LINE entity: collect LINE ids, then check
# each EDGE_CURVE geometry reference against them
string(REGEX MATCHALL "#([0-9]+) = LINE" line_defs "${content}")
set(line_ids "")
foreach(def ${line_defs})
	string(REGEX REPLACE "#([0-9]+) = LINE" "\\1" line_id "${def}")
	list(APPEND line_ids "${line_id}")
endforeach()

string(REGEX MATCHALL "EDGE_CURVE\\('[^']*', #[0-9]+, #[0-9]+,#([0-9]+)," edge_refs "${content}")
if(edge_refs STREQUAL "")
	message(FATAL_ERROR "no EDGE_CURVE entities found in output")
endif()
foreach(ref ${edge_refs})
	string(REGEX REPLACE ".*,#([0-9]+),$" "\\1" geom_id "${ref}")
	if(NOT geom_id IN_LIST line_ids)
		message(FATAL_ERROR "EDGE_CURVE references #${geom_id} which is not a LINE")
	endif()
endforeach()
