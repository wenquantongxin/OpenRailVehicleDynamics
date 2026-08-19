# Checks a built candidate's declared dynamic load-time closure for shared Drake.
#
# On the built artefact, not only on the CMake graph. A graph check has to model
# every way CMake can be told about a dependency — a library selection option in
# any of its spellings, a generator expression evaluated per configuration, an
# imported target declared in a scope where it is invisible, a link edge written
# from a different directory — and each is a separate thing to get right.
# Deferring a graph check can move it later, but still does not turn the
# configure-time representation into the built program's resolved load-time
# closure.
#
# CMake resolves the linked file's declared dynamic dependencies recursively
# using the platform rules available in this test environment. Whatever CMake
# spelling produced them, a shared `libdrake` either appears in that resolved
# closure or it does not. This is not a claim about static archives, `dlopen`,
# `LoadLibrary`, `LD_PRELOAD`, a changed loader search environment, or a library
# deliberately renamed to hide its identity. The product graph gate rejects
# ordinary static and shared Drake inputs early; provenance and object ownership
# say what the product embeds statically. This check has the narrower job of
# keeping a shared Drake out of this candidate's declared runtime closure.
#
# What this permits, and must: the candidate links the landed tree statically.
# That is vendored Drake source compiled into ORVD's own archives, and it is what
# the product is. What it must not do is load the installed shared library as
# well — that library and the vendored copy export the same `drake::` symbols, so
# both in one address space is an ODR violation whose likely symptom is a
# comparison that appears to pass.
cmake_minimum_required(VERSION 3.24)

if(NOT DEFINED CANDIDATE_EXECUTABLE)
    message(FATAL_ERROR "CANDIDATE_EXECUTABLE was not given")
endif()
if(NOT EXISTS "${CANDIDATE_EXECUTABLE}")
    message(FATAL_ERROR
        "the candidate '${CANDIDATE_EXECUTABLE}' does not exist, so this check "
        "would pass by having nothing to look at")
endif()

# Keep the candidate-facing variable and diagnostics above, but use the same
# implementation as the relocated-install check.  Windows API-set handling,
# conflict reporting and runtime search semantics must not diverge between two
# executables merely because one is built in-tree and one is installed.
set(CHECKED_EXECUTABLE "${CANDIDATE_EXECUTABLE}")
include("${CMAKE_CURRENT_LIST_DIR}/../installation/verify_runtime_dependencies.cmake")
