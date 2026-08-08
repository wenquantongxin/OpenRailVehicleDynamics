#include "orvd/configuration/gz18_wheel_rail_contact.h"

#include <filesystem>
#include <memory>
#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::configuration::AssembleGz18WheelRailContact(
                  std::declval<const std::filesystem::path&>(),
                  std::declval<const orvd::configuration::StartupWheelRailBinding&>(),
                  0.0)),
              std::unique_ptr<orvd::configuration::Gz18WheelRailContact>>);
