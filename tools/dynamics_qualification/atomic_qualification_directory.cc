#include "atomic_qualification_directory.h"

#include <system_error>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::dynamics_qualification {

AtomicQualificationDirectory::AtomicQualificationDirectory(
    std::filesystem::path final_path)
    : final_path_(std::move(final_path)) {
    if (final_path_.empty() || final_path_.filename().empty()) {
        throw std::invalid_argument(
            "qualification output directory must name a final directory");
    }

    const std::filesystem::path parent = final_path_.has_parent_path()
                                             ? final_path_.parent_path()
                                             : std::filesystem::path(".");
    if (!std::filesystem::is_directory(parent)) {
        throw std::invalid_argument(
            "qualification output parent '" + parent.string() +
            "' is not an existing directory");
    }
    if (std::filesystem::exists(final_path_)) {
        throw std::invalid_argument(
            "qualification output path '" + final_path_.string() +
            "' already exists");
    }

    working_path_ = parent / (final_path_.filename().string() + ".partial");
    if (std::filesystem::exists(working_path_)) {
        throw std::invalid_argument(
            "qualification partial path '" + working_path_.string() +
            "' already exists");
    }
    if (!std::filesystem::create_directory(working_path_)) {
        throw std::runtime_error(
            "qualification partial directory '" + working_path_.string() +
            "' could not be created");
    }
    owns_working_directory_ = true;
}

AtomicQualificationDirectory::~AtomicQualificationDirectory() {
    if (owns_working_directory_ && !published_) {
        std::error_code ignored;
        std::filesystem::remove_all(working_path_, ignored);
    }
}

void AtomicQualificationDirectory::Publish() {
    if (published_ || !owns_working_directory_) {
        throw std::logic_error(
            "qualification output directory has already been published");
    }
    if (std::filesystem::exists(final_path_)) {
        throw std::runtime_error(
            "qualification output path '" + final_path_.string() +
            "' appeared before publication; it was not overwritten");
    }
    std::filesystem::rename(working_path_, final_path_);
    published_ = true;
    owns_working_directory_ = false;
}

}  // namespace orvd::dynamics_qualification
