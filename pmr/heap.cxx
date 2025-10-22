// local includes
#include <gl_util.h>

// implemented header
#include "pmr/heap.h"


namespace otv::pmr {

heap::heap(std::span<std::byte> memory, size_t granularity)
	: memory_region{memory}
	, _buddy_alloc {
		memory,
		std::min<uint8_t>(std::bit_width(memory.size()) - 1/*floor(log2)*/, 12)
	}
	, _pool_alloc {
		&_buddy_alloc,
		size_t{1} << _buddy_alloc.base_order(),
		static_cast<uint8_t>(std::bit_width(std::min(memory.size(), size_t{1}) - 1)/*ceil(log2)*/)
	}
{}

void heap::clear ()
{
	_pool_alloc.leak();
	_buddy_alloc.clear();
}

} // namespace otv::pmr
