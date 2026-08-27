#!/usr/bin/env python3
"""Exercise the strict floating-point CMake gate and its exact diagnostics."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import tempfile


def run_probe(
    cmake: Path,
    probe: Path,
    *,
    base_flags: str = "",
    release_flags: str = "",
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            str(cmake),
            "-DCMAKE_BUILD_TYPE=Release",
            f"-DCMAKE_CXX_FLAGS={base_flags}",
            f"-DCMAKE_CXX_FLAGS_RELEASE={release_flags}",
            "-P",
            str(probe),
        ],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cmake", type=Path, required=True)
    parser.add_argument("--cxx-compiler", type=Path, required=True)
    parser.add_argument("--osx-sysroot", default="")
    parser.add_argument("--source-root", type=Path, required=True)
    arguments = parser.parse_args()

    module_directory = (arguments.source_root / "cmake").resolve(strict=True)
    with tempfile.TemporaryDirectory(
        prefix="orvd-strict-floating-point-gate-"
    ) as temporary:
        probe = Path(temporary) / "probe.cmake"
        probe.write_text(
            "list(APPEND CMAKE_MODULE_PATH \""
            + module_directory.as_posix()
            + "\")\n"
            "include(OrvdStrictFloatingPointQualification)\n"
            "orvd_require_strict_floating_point_qualification()\n",
            encoding="utf-8",
        )

        for flags in (
            "",
            "-O3 -fno-fast-math",
            "-ffp-contract=fast",
            "-ffp-contract=off",
        ):
            result = run_probe(
                arguments.cmake, probe, release_flags=flags
            )
            if result.returncode != 0:
                raise RuntimeError(
                    f"strict floating-point flags were rejected: {flags!r}\n"
                    f"{result.stdout}"
                )

        forbidden_cases = (
            ("-ffast-math", "CMAKE_CXX_FLAGS"),
            ("-Ofast", "CMAKE_CXX_FLAGS_RELEASE"),
            ("-ffinite-math-only", "CMAKE_CXX_FLAGS_RELEASE"),
            ("-ffp-model=fast", "CMAKE_CXX_FLAGS_RELEASE"),
        )
        for token, variable in forbidden_cases:
            result = run_probe(
                arguments.cmake,
                probe,
                base_flags=token if variable == "CMAKE_CXX_FLAGS" else "",
                release_flags=(
                    token if variable == "CMAKE_CXX_FLAGS_RELEASE" else ""
                ),
            )
            expected = (
                "ORVD strict floating-point qualification rejected "
                f"{variable} token '{token}'"
            )
            normalized_output = " ".join(result.stdout.split())
            if result.returncode == 0 or expected not in normalized_output:
                raise RuntimeError(
                    f"unsafe token {token!r} did not produce its exact "
                    f"qualification rejection\n{result.stdout}"
                )

        target_probe_source = Path(temporary) / "target-option-probe"
        target_probe_source.mkdir()
        (target_probe_source / "main.cc").write_text(
            "int main() { return 0; }\n", encoding="utf-8"
        )
        (target_probe_source / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.24)\n"
            "project(orvd_strict_fp_target_probe LANGUAGES CXX)\n"
            "list(APPEND CMAKE_MODULE_PATH \""
            + module_directory.as_posix()
            + "\")\n"
            "include(OrvdStrictFloatingPointQualification)\n"
            "orvd_require_strict_floating_point_qualification()\n"
            "if(PROBE_MODE STREQUAL \"existing-launcher\")\n"
            "  set(CMAKE_CXX_COMPILER_LAUNCHER \"${CMAKE_COMMAND};-E;env\")\n"
            "endif()\n"
            "orvd_enable_strict_floating_point_compile_launcher()\n"
            "if(PROBE_MODE STREQUAL \"unsafe-target\")\n"
            "  add_compile_options(-ffp-model=fast)\n"
            "endif()\n"
            "add_executable(target_option_probe main.cc)\n"
            "if(PROBE_MODE STREQUAL \"safe-response\")\n"
            "  file(WRITE \"${CMAKE_BINARY_DIR}/safe.rsp\" \"-O2\")\n"
            "  target_compile_options(target_option_probe PRIVATE \"@safe.rsp\")\n"
            "elseif(PROBE_MODE STREQUAL \"unsafe-response\")\n"
            "  file(WRITE \"${CMAKE_BINARY_DIR}/unsafe.rsp\" \"-ffast-math\")\n"
            "  target_compile_options(target_option_probe PRIVATE \"@unsafe.rsp\")\n"
            "elseif(PROBE_MODE STREQUAL \"opaque-response\")\n"
            "  target_compile_options(target_option_probe PRIVATE \"@missing.rsp\")\n"
            "elseif(PROBE_MODE STREQUAL \"unsafe-xclang\")\n"
            "  target_compile_options(target_option_probe PRIVATE \"-Xclang=-ffast-math\")\n"
            "elseif(PROBE_MODE STREQUAL \"unsafe-forced-include\")\n"
            "  file(WRITE \"${CMAKE_BINARY_DIR}/forced.h\" \"#undef __FAST_MATH__\\n\")\n"
            "  target_compile_options(target_option_probe PRIVATE \"-include${CMAKE_BINARY_DIR}/forced.h\")\n"
            "elseif(PROBE_MODE STREQUAL \"safe-contract-fast\")\n"
            "  target_compile_options(target_option_probe PRIVATE -ffp-contract=fast)\n"
            "elseif(PROBE_MODE STREQUAL \"safe-contract-off\")\n"
            "  target_compile_options(target_option_probe PRIVATE -ffp-contract=off)\n"
            "endif()\n",
            encoding="utf-8",
        )

        def configure_target_probe(
            mode: str,
        ) -> tuple[Path, subprocess.CompletedProcess[str]]:
            build_directory = Path(temporary) / f"target-option-{mode}"
            configure_command = [
                str(arguments.cmake),
                "-S",
                str(target_probe_source),
                "-B",
                str(build_directory),
                f"-DCMAKE_CXX_COMPILER={arguments.cxx_compiler}",
                f"-DPROBE_MODE={mode}",
            ]
            if arguments.osx_sysroot:
                configure_command.append(
                    f"-DCMAKE_OSX_SYSROOT={arguments.osx_sysroot}"
                )
            result = subprocess.run(
                configure_command,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            return build_directory, result

        def build_target_probe(
            build_directory: Path,
        ) -> subprocess.CompletedProcess[str]:
            return subprocess.run(
                [str(arguments.cmake), "--build", str(build_directory)],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )

        target_probe_build, configure = configure_target_probe(
            "unsafe-target"
        )
        if configure.returncode != 0:
            raise RuntimeError(
                "target-option compile-launcher probe did not configure\n"
                + configure.stdout
            )
        build = build_target_probe(target_probe_build)
        expected = (
            "ORVD strict floating-point compile command rejected token "
            "'-ffp-model=fast'"
        )
        normalized_output = " ".join(build.stdout.split())
        if build.returncode == 0 or expected not in normalized_output:
            raise RuntimeError(
                "directory/target option bypassed the final compile-command "
                "audit\n" + build.stdout
            )

        safe_response_build, configure = configure_target_probe(
            "safe-response"
        )
        if configure.returncode != 0:
            raise RuntimeError(
                "safe response-file probe did not configure\n"
                + configure.stdout
            )
        build = build_target_probe(safe_response_build)
        if build.returncode != 0:
            raise RuntimeError(
                "readable safe response file was rejected\n" + build.stdout
            )

        for mode in ("safe-contract-fast", "safe-contract-off"):
            contraction_build, configure = configure_target_probe(mode)
            if configure.returncode != 0:
                raise RuntimeError(
                    f"{mode} probe did not configure\n{configure.stdout}"
                )
            build = build_target_probe(contraction_build)
            if build.returncode != 0:
                raise RuntimeError(
                    "an admissible floating-point contraction policy was "
                    f"rejected by {mode}\n{build.stdout}"
                )

        for mode, token in (
            ("unsafe-response", "-ffast-math"),
            ("opaque-response", "@missing.rsp"),
            ("unsafe-xclang", "-Xclang=-ffast-math"),
            ("unsafe-forced-include", "-include"),
        ):
            response_build, configure = configure_target_probe(mode)
            if configure.returncode != 0:
                raise RuntimeError(
                    f"{mode} probe did not configure\n{configure.stdout}"
                )
            build = build_target_probe(response_build)
            normalized_output = " ".join(build.stdout.split())
            if mode in ("unsafe-response", "unsafe-xclang"):
                expected = (
                    "ORVD strict floating-point compile command rejected "
                    f"token '{token}'"
                )
            elif mode == "opaque-response":
                expected = (
                    "ORVD strict floating-point compile command rejected "
                    f"opaque response token '{token}'"
                )
            else:
                expected = (
                    "ORVD strict floating-point compile command rejected "
                    "token '-include"
                )
            if build.returncode == 0 or expected not in normalized_output:
                raise RuntimeError(
                    f"{mode} bypassed its exact rejection\n{build.stdout}"
                )

        compile_launcher = (
            module_directory / "OrvdStrictFloatingPointCompileLauncher.cmake"
        )

        def run_compile_launcher(
            compiler_id: str, forwarded_tokens: tuple[str, ...]
        ) -> subprocess.CompletedProcess[str]:
            return subprocess.run(
                [
                    str(arguments.cmake),
                    "-DORVD_STRICT_FLOATING_POINT_CXX_COMPILER_ID:STRING="
                    + compiler_id,
                    "-P",
                    str(compile_launcher),
                    "--",
                    str(arguments.cmake),
                    "-E",
                    "echo",
                    *forwarded_tokens,
                ],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                cwd=temporary,
            )

        safe_apple_openmp = run_compile_launcher(
            "AppleClang", ("-Xclang", "-fopenmp")
        )
        if safe_apple_openmp.returncode != 0:
            raise RuntimeError(
                "the exact AppleClang OpenMP forwarding pair was rejected\n"
                + safe_apple_openmp.stdout
            )

        safe_apple_openmp_response = Path(temporary) / "apple-openmp.rsp"
        safe_apple_openmp_response.write_text(
            "-Xclang -fopenmp", encoding="utf-8"
        )
        safe_response = run_compile_launcher(
            "AppleClang", ("@apple-openmp.rsp",)
        )
        if safe_response.returncode != 0:
            raise RuntimeError(
                "the exact AppleClang OpenMP pair in a readable response "
                "file was rejected\n" + safe_response.stdout
            )

        for compiler_id, forwarded_tokens in (
            ("GNU", ("-Xclang", "-fopenmp")),
            ("Clang", ("-Xclang", "-fopenmp")),
            ("AppleClang", ("-Xclang",)),
            ("AppleClang", ("-Xclang", "-ffast-math")),
            ("AppleClang", ("-Xclang=-fopenmp",)),
        ):
            rejected = run_compile_launcher(compiler_id, forwarded_tokens)
            expected = (
                "ORVD strict floating-point compile command rejected token "
                f"'{forwarded_tokens[0]}'"
            )
            normalized_output = " ".join(rejected.stdout.split())
            if rejected.returncode == 0 or expected not in normalized_output:
                raise RuntimeError(
                    "an unapproved compiler forwarding form bypassed the "
                    "compile-command audit\n" + rejected.stdout
                )

        for response_contents in (
            "-Xclang",
            "-Xclang -ffast-math",
        ):
            unsafe_apple_response = Path(temporary) / "unsafe-apple-openmp.rsp"
            unsafe_apple_response.write_text(
                response_contents, encoding="utf-8"
            )
            rejected = run_compile_launcher(
                "AppleClang", ("@unsafe-apple-openmp.rsp",)
            )
            expected = (
                "ORVD strict floating-point compile command rejected token "
                "'-Xclang'"
            )
            normalized_output = " ".join(rejected.stdout.split())
            if rejected.returncode == 0 or expected not in normalized_output:
                raise RuntimeError(
                    "an unsafe AppleClang response-file forwarding form "
                    "bypassed the compile-command audit\n" + rejected.stdout
                )

        rejected = run_compile_launcher("GNU", ("@apple-openmp.rsp",))
        expected = (
            "ORVD strict floating-point compile command rejected token "
            "'-Xclang'"
        )
        normalized_output = " ".join(rejected.stdout.split())
        if rejected.returncode == 0 or expected not in normalized_output:
            raise RuntimeError(
                "an AppleClang-only OpenMP response pair was admitted for "
                "GNU\n" + rejected.stdout
            )

        split_apple_response = Path(temporary) / "split-apple-openmp.rsp"
        split_apple_response.write_text("-Xclang", encoding="utf-8")
        rejected = run_compile_launcher(
            "AppleClang", ("@split-apple-openmp.rsp", "-fopenmp")
        )
        normalized_output = " ".join(rejected.stdout.split())
        if rejected.returncode == 0 or expected not in normalized_output:
            raise RuntimeError(
                "an AppleClang OpenMP pair split across a response-file "
                "boundary bypassed the compile-command audit\n"
                + rejected.stdout
            )

        _, configure = configure_target_probe("existing-launcher")
        expected = (
            "ORVD strict floating-point qualification rejects a "
            "pre-existing CMAKE_CXX_COMPILER_LAUNCHER"
        )
        normalized_output = " ".join(configure.stdout.split())
        if configure.returncode == 0 or expected not in normalized_output:
            raise RuntimeError(
                "pre-existing compiler launcher bypassed rejection\n"
                + configure.stdout
            )

        unsupported_generator_probe = (
            Path(temporary) / "unsupported-generator.cmake"
        )
        unsupported_generator_probe.write_text(
            "set(CMAKE_GENERATOR \"Xcode\")\n"
            "list(APPEND CMAKE_MODULE_PATH \""
            + module_directory.as_posix()
            + "\")\n"
            "include(OrvdStrictFloatingPointQualification)\n"
            "orvd_enable_strict_floating_point_compile_launcher()\n",
            encoding="utf-8",
        )
        generator_result = subprocess.run(
            [str(arguments.cmake), "-P", str(unsupported_generator_probe)],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        expected = (
            "ORVD strict floating-point qualification requires a Ninja or "
            "Makefile generator"
        )
        normalized_output = " ".join(generator_result.stdout.split())
        if (
            generator_result.returncode == 0
            or expected not in normalized_output
        ):
            raise RuntimeError(
                "unsupported generator bypassed rejection\n"
                + generator_result.stdout
            )

    print("Strict floating-point qualification gate verification passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
