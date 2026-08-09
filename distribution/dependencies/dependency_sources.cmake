orvd_declare_dependency(
    KEY eigen
    NAME Eigen
    VERSION 3.4.0
    ARCHIVE eigen-3.4.0.tar.gz
    SOURCE_DIRECTORY eigen-3.4.0
    SOURCE_URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    LICENSE_PATH COPYING.APACHE
    LICENSE_PATH COPYING.BSD
    LICENSE_PATH COPYING.GPL
    LICENSE_PATH COPYING.LGPL
    LICENSE_PATH COPYING.MINPACK
    LICENSE_PATH COPYING.MPL2
    LICENSE_PATH COPYING.README)

orvd_declare_dependency(
    KEY fmt
    NAME fmt
    VERSION 9.1.0
    ARCHIVE fmt-9.1.0.tar.gz
    SOURCE_DIRECTORY fmt-9.1.0
    SOURCE_URL https://github.com/fmtlib/fmt/archive/refs/tags/9.1.0.tar.gz
    LICENSE_PATH LICENSE.rst)

orvd_declare_dependency(
    KEY nlohmann_json
    NAME nlohmann/json
    VERSION 3.12.0
    ARCHIVE nlohmann-json-3.12.0.tar.xz
    SOURCE_DIRECTORY json
    SOURCE_URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    LICENSE_PATH LICENSE.MIT)

orvd_declare_dependency(
    KEY sundials
    NAME SUNDIALS
    VERSION 7.7.0
    ARCHIVE sundials-7.7.0.tar.gz
    SOURCE_DIRECTORY sundials-7.7.0
    SOURCE_URL https://github.com/LLNL/sundials/releases/download/v7.7.0/sundials-7.7.0.tar.gz
    LICENSE_PATH LICENSE
    LICENSE_PATH NOTICE)
