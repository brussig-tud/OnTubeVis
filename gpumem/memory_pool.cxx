#include "memory_pool.inl"


namespace otv::gpumem {

bool memory_pool_alloc::create (size_type num_blocks, size_type block_length, span<std::byte> memory)
{
	// Set members.
	_block_length = block_length;
	_memory       = memory;

	// Generate blocks.
	_free_blocks.clear();
	_free_blocks.reserve(num_blocks);

	for (auto i {num_blocks - 1}; i != -1; --i) {
		_free_blocks.emplace_back(_memory.data() + i * block_length);
	}

	return true;
}

} // namespace otv::gpumem
