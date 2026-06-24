#include "buffer.h"

#include "gl_util.h"


namespace otv::gpumem {

void free_buffer (handle_type handle)
{
	if (handle == 0) return;

	glDeleteBuffers(1, &handle);
	check_gl_errors("gpumem::free_buffer");
}

auto buffer::create() -> buffer
{
	handle_type handle;
	glCreateBuffers(1, &handle);
	check_gl_errors("gpumem::buffer::create");
	if (!handle) throw;
	return {std::move(handle)};
}

} // namespace otv::gpumem

