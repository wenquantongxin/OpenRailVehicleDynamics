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

        for flags in ("", "-O3 -fno-fast-math"):
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
            "endif()\n",
            encoding="utf-8",
        )

        def configure_target_probe(
            mode: str,
        ) -> tuple[Path, subprocess.CompletedProcess[str]]:
            build_directory = Path(temporary) / f"target-option-{mode}"
            result = subprocess.run(
                [
                    str(arguments.cmake),
                    "-S",
                    str(target_probe_source),
                    "-B",
                    str(build_directory),
                    f"-DCMAKE_CXX_COMPILER={arguments.cxx_compiler}",
                    f"-DPROBE_MODE={mode}",
                ],
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
