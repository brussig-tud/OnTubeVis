#include "memory_pool.inl"


namespace otv::gpumem {

bool memory_pool_alloc::create (size_type num_blocks, size_type block_size, span<std::byte> memory)
{
	// Set members.
	_block_size = block_size;
	_memory     = memory;

	// Generate blocks.
	_free_blocks.clear();
	_free_blocks.reserve(num_blocks);

	for (auto i {num_blocks - 1}; i != -1; --i) {
		_free_blocks.emplace_back(_memory.data() + i * block_size);
	}

	return true;
}

} // namespace otv::gpumem
