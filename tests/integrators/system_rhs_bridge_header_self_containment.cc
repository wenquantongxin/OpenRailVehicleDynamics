#include "orvd/integrators/system_rhs_bridge.h"

// This translation unit intentionally includes no system-assembly definition.
// It proves that deleting through the public bridge type does not instantiate
// unique_ptr's deleter while SystemRuntimeContext is incomplete.
void DeleteSystemRhsBridge(orvd::integrators::SystemRhsBridge* bridge) {
    delete bridge;
}
