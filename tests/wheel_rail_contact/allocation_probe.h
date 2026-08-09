#pragma once

#include <cstddef>
#include <cstdlib>
#include <new>

// Counts every heap allocation the program makes, so a test can assert that a
// stretch of work made none.
//
// The contact evaluation runs inside an integrator's right-hand side, thousands
// of times per simulated second. An allocation there is not a correctness
// problem and no assertion about results will ever catch one; it is a
// performance problem that only shows up as a profile, long after the code that
// caused it was written. So it is checked directly.
//
// This replaces the global allocation functions, which a program may do exactly
// once. Include this header in exactly one translation unit per test binary.
//
// What it counts is allocations, not bytes, and it counts them wherever they
// happen — including inside the standard library. That is the point: a
// `std::vector` grown inside a solver is exactly what this is looking for.

namespace orvd::test {

inline std::size_t allocation_count = 0;

// A scope that remembers the count on entry so a test can ask what happened
// inside it. Nested scopes are independent.
class AllocationScope {
  public:
    AllocationScope() : entry_count_(allocation_count) {}

    [[nodiscard]] std::size_t allocations() const {
        return allocation_count - entry_count_;
    }

  private:
    std::size_t entry_count_;
};

}  // namespace orvd::test

void* operator new(std::size_t size) {
    ++orvd::test::allocation_count;
    // Zero-sized allocations must still return distinct pointers, and malloc is
    // permitted to return null for them.
    void* memory = std::malloc(size == 0 ? 1 : size);
    if (memory == nullptr) {
        throw std::bad_alloc();
    }
    return memory;
}

void* operator new[](std::size_t size) { return operator new(size); }

void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept {
    std::free(memory);
}
void operator delete[](void* memory, std::size_t) noexcept {
    std::free(memory);
}
