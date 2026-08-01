#pragma once

#if defined(_WIN32)
#if defined(ORVD_DRAKE_MARKER_BUILD)
#define ORVD_DRAKE_MARKER_EXPORT __declspec(dllexport)
#else
#define ORVD_DRAKE_MARKER_EXPORT __declspec(dllimport)
#endif
#else
#define ORVD_DRAKE_MARKER_EXPORT
#endif

extern "C" ORVD_DRAKE_MARKER_EXPORT int orvd_drake_marker_identity();
