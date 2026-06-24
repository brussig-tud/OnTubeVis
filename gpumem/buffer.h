#pragma once

#include "common.h"
#include "util.h"


namespace otv::gpumem {

/// Destroy a GL buffer object.
void free_buffer (handle_type);

class buffer : public RAII<handle_type, free_buffer>
{
public:
	[[nodiscard]] static auto create () -> buffer;
};

} // namespace otv::gpumem

