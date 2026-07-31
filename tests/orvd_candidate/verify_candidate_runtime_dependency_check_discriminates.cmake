# Checks that the candidate's shared-Drake runtime-dependency check fails, and
# fails for the stated reason.
#
# `WILL_FAIL` alone would accept any non-zero exit, so the check would look
# discriminating while actually passing because the file was missing, or because
# a typo made the script abort before it looked at anything. Each case below
# names the diagnostic it expects.
cmake_minimum_required(VERSION 3.24)

function(expect_refusal label executable expected_fragment)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -DCANDIDATE_EXECUTABLE=${executable}
                -P ${CHECK_SCRIPT}
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE errors)
    if(result EQUAL 0)
        message(SEND_ERROR "${label}: the check accepted what it must refuse")
        return()
    endif()
    if(NOT "${output}${errors}" MATCHES "${expected_fragment}")
        message(SEND_ERROR
            "${label}: the check refused, but for another reason than "
            "'${expected_fragment}':\n${output}${errors}")
    endif()
endfunction()

# A program that really does load Drake.
expect_refusal("a Drake-loading program" "${DRAKE_LOADING_EXECUTABLE}"
               "runtime dependency closure contains shared Drake")

# A program whose dependency cannot be resolved. Behind an unresolved node the
# closure was never walked, so a Drake there would go unmentioned.
expect_refusal("an unresolvable dependency" "${UNRESOLVABLE_EXECUTABLE}"
               "could not all be resolved")

# The same, with a name CMake reads as false. A check that asked whether the list
# "is true" rather than how long it is would decide there was nothing there.
expect_refusal("a false-like dependency name" "${FALSE_LIKE_EXECUTABLE}"
               "could not all be resolved")

# And a file that is not there at all, which must not read as "nothing to see".
expect_refusal("a missing file" "${CMAKE_CURRENT_LIST_DIR}/no_such_program"
               "does not exist")

message(STATUS
    "the candidate shared-Drake check refuses each case for its own reason")
