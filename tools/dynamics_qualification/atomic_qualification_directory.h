#pragma once

#include <filesystem>

namespace orvd::dynamics_qualification {

// One unpublished qualification artefact directory.
//
// Construction creates a sibling `<final-name>.partial` directory. The caller
// writes and closes every file below `working_path()` and then calls Publish(),
// which performs one same-parent rename. A failed or abandoned instance removes
// only the partial directory it created. Existing partial and final paths are
// refused and are never removed or overwritten.
class AtomicQualificationDirectory final {
   public:
    explicit AtomicQualificationDirectory(
        std::filesystem::path final_path);
    ~AtomicQualificationDirectory();

    AtomicQualificationDirectory(const AtomicQualificationDirectory&) = delete;
    AtomicQualificationDirectory& operator=(
        const AtomicQualificationDirectory&) = delete;
    AtomicQualificationDirectory(AtomicQualificationDirectory&&) = delete;
    AtomicQualificationDirectory& operator=(AtomicQualificationDirectory&&) =
        delete;

    [[nodiscard]] const std::filesystem::path& final_path() const noexcept {
        return final_path_;
    }
    [[nodiscard]] const std::filesystem::path& working_path() const noexcept {
        return working_path_;
    }
    [[nodiscard]] bool published() const noexcept { return published_; }

    void Publish();

   private:
    std::filesystem::path final_path_;
    std::filesystem::path working_path_;
    bool owns_working_directory_{false};
    bool published_{false};
};

}  // namespace orvd::dynamics_qualification
