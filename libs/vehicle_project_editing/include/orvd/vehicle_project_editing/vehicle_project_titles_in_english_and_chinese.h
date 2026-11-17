#pragma once

#include <string>

// What an interface shows people for a row of the model tree, an element kind
// or a section of properties: a title, or a short note, in the two languages
// it shows. The editing layer states every such text; an interface states
// none.

namespace orvd::vehicle_project_editing {

struct Title {
    std::string en;
    std::string zh;

    friend bool operator==(const Title&, const Title&) = default;
};

}  // namespace orvd::vehicle_project_editing
