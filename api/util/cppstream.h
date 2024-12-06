
////
// INTERNAL HEADER - DO NOT INCLUDE!!!!!


#ifndef __CPPSTREAM_H__
#define __CPPSTREAM_H__


//////
//
// Includes
//

// C++ STL
#include <iostream>



//////
//
// Functions
//

template <class T>
struct hex {
	const T& val;
	hex(const T& val) : val(val) {}
};

template <class T>
std::ostream& operator << (std::ostream& os, const hex<T>& h)
{
	auto f_bak = os.flags();
	os << std::hex << std::uppercase << uintptr_t(h.val);
	os.flags(f_bak);
	return os;
}



#endif // ifdef __CPPSTREAM_H__
