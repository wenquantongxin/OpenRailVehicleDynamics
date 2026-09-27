#include "orvd/scene_record/scene_record_writer.h"

#include <type_traits>

static_assert(!std::is_copy_constructible_v<
              orvd::scene_record::SceneRecordWriter>);
static_assert(!std::is_move_constructible_v<
              orvd::scene_record::SceneRecordWriter>);
