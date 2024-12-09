
////
// INTERNAL HEADER - DO NOT INCLUDE!!!!!


#ifndef __MODULE_HANDLING_H__
#define __MODULE_HANDLING_H__


//////
//
// Includes
//

// C++ STL
#include <string>



//////
//
// Structs and typedefs
//

// The main function signature
typedef int(*main_funct)(int, char**);



//////
//
// Functions
//

/**
 * @brief Retrieve the address of the function behind the @c main symbol defined <em>in this very library</em>.
 *
 * This is guaranteed to not return the the address of any other @c main symbol from either the calling executable or
 * other modules loaded into the process.
 *
 * @return The function pointer to <em>our</em> @c main function.
 */
main_funct get_on_tube_vis_main (void);

/**
 * @brief Report the filepath of the shared object / DLL implementing the OnTubeVis service that was actually loaded.
 *
 * @return The function pointer to <em>our</em> @c main function.
 */
const std::string& on_tube_vis_module_filepath (void);


#endif // ifdef __MODULE_HANDLING_H__
