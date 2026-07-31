# Proves that the cross-process comparison cannot quietly become "Drake versus
# Drake" through a wiring mistake.
#
# A non-zero result alone is not enough: a missing executable or a launcher typo
# would also fail. The diagnostic must say that the two roles resolved to the
# same file.
cmake_minimum_required(VERSION 3.24)

foreach(required_variable IN ITEMS COMPARISON_EXECUTABLE EMITTER_EXECUTABLE)
    if(NOT DEFINED ${required_variable})
        message(FATAL_ERROR "${required_variable} was not given")
    endif()
endforeach()

execute_process(
    COMMAND "${COMPARISON_EXECUTABLE}"
            "${EMITTER_EXECUTABLE}" "${EMITTER_EXECUTABLE}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE errors)

if(result EQUAL 0)
    message(FATAL_ERROR
        "the comparison accepted one executable in both the reference and "
        "candidate roles")
endif()
if(NOT "${output}${errors}" MATCHES
       "reference and candidate emitters must be different executables")
    message(FATAL_ERROR
        "the comparison failed, but not because the two roles named the same "
        "file:\n${output}${errors}")
endif()

message(STATUS
    "the cross-process comparison refuses one emitter in both roles")
