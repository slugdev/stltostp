# Regression check for https://github.com/slugdev/stltostp issue #1 "Scale of
# objects is 1000x bigger": the STEP file must declare millimetres by default
# (a .METRE. declaration makes importers show the model 1000x too big) and the
# coordinates must be written unscaled, exactly as read from the stl file.
#
# Usage: cmake -DEXE=<stltostp> -DDIR=<test dir> -P check_units_scale.cmake

cmake_minimum_required(VERSION 3.12)

function(convert out_file)
	execute_process(
		COMMAND "${EXE}" "${DIR}/cube.stl" "${DIR}/${out_file}" ${ARGN}
		RESULT_VARIABLE res
	)
	if(NOT res EQUAL 0)
		message(FATAL_ERROR "stltostp failed with exit code ${res}")
	endif()
endfunction()

# the cube.stl model is a unit cube, so every coordinate is 0 or 1 regardless
# of the requested unit; anything else means the geometry got scaled
function(check_unit_cube_coords content)
	string(REGEX MATCHALL "CARTESIAN_POINT\\('', \\([^)]*\\)\\)" points "${content}")
	if(points STREQUAL "")
		message(FATAL_ERROR "no CARTESIAN_POINT entities found in output")
	endif()
	foreach(point ${points})
		if(NOT point MATCHES "^CARTESIAN_POINT\\('', \\([01],[01],[01]\\)\\)$")
			message(FATAL_ERROR "unit cube coordinates were scaled: ${point}")
		endif()
	endforeach()
endfunction()

# default output: millimetres, coordinates unchanged
convert(cube_units_default.stp)
file(READ "${DIR}/cube_units_default.stp" content)
if(NOT content MATCHES "LENGTH_UNIT\\(\\) NAMED_UNIT\\(\\*\\) SI_UNIT\\(\\.MILLI\\.,\\.METRE\\.\\)")
	message(FATAL_ERROR "default output does not declare millimetre length units")
endif()
check_unit_cube_coords("${content}")

# the length unit must be assigned to the context used by the shape
# representation, otherwise importers fall back to their own default unit
string(REGEX MATCH "#([0-9]+) = \\( LENGTH_UNIT\\(\\) NAMED_UNIT\\(\\*\\) SI_UNIT\\(\\.MILLI\\." unit_def "${content}")
set(unit_id "${CMAKE_MATCH_1}")
string(REGEX MATCH "#([0-9]+) = \\( GEOMETRIC_REPRESENTATION_CONTEXT[^\n]*GLOBAL_UNIT_ASSIGNED_CONTEXT\\(\\(#${unit_id}," context_def "${content}")
if(context_def STREQUAL "")
	message(FATAL_ERROR "length unit #${unit_id} is not assigned to a geometric representation context")
endif()
set(context_id "${CMAKE_MATCH_1}")
if(NOT content MATCHES "SHAPE_REPRESENTATION\\('',[^\n]*#${context_id}\\)")
	message(FATAL_ERROR "shape representation does not reference the unit context #${context_id}")
endif()

# explicit units only change the declaration, never the coordinate values
convert(cube_units_cm.stp units cm)
file(READ "${DIR}/cube_units_cm.stp" content)
if(NOT content MATCHES "SI_UNIT\\(\\.CENTI\\.,\\.METRE\\.\\)")
	message(FATAL_ERROR "cm output does not declare centimetre length units")
endif()
check_unit_cube_coords("${content}")

convert(cube_units_m.stp units m)
file(READ "${DIR}/cube_units_m.stp" content)
if(NOT content MATCHES "SI_UNIT\\(\\$,\\.METRE\\.\\)")
	message(FATAL_ERROR "m output does not declare metre length units")
endif()
check_unit_cube_coords("${content}")

convert(cube_units_in.stp units in)
file(READ "${DIR}/cube_units_in.stp" content)
if(NOT content MATCHES "CONVERSION_BASED_UNIT\\('INCH'")
	message(FATAL_ERROR "inch output does not declare an inch conversion based unit")
endif()
check_unit_cube_coords("${content}")
