
////
// INTERNAL HEADER - DO NOT INCLUDE!!!!!


#ifndef __UID_H__
#define __UID_H__


//////
//
// Includes
//

// C++ STL
#include <atomic>



//////
//
// Functions
//

/**
 * @brief
 *		Returns a unique ID, i.e. what this function returns while the module remains loaded will never repeat until the
 *		underlying data type overflows. The implementation is thread-safe.
 *
 * @return A unique ID of the desired type.
 */
template <class T>
inline T get_unique_id (void)
{
	static std::atomic<T> counter{0};
    return counter++;
}


#endif // ifdef __UID_H__
