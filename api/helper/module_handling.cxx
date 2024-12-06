
//////
//
// Includes
//

// C++ STL
#include <string>
#include <cstring>
#include <cassert>

// Platform SDKs
#ifdef _WIN32
	#define NOMINMAX // we don't want the "min" and "max" macros
	#include <windows.h>
	#include <Psapi.h>
#else
	#include <link.h>
	#include <dlfcn.h>
#endif

// Public interface
#include <OnTubeVis/OnTubeVis.h>

// Private interface
#ifdef _WIN32
	#include <apimain.h>
#endif

// Local includes
#include "module_handling.h"



//////
//
// Forwards declarations
//

#ifndef _WIN32
	// Callback for the enumeration of modules loaded by the OS.
	int module_iteration_callback (struct dl_phdr_info* info, size_t size, void* data);
#endif



//////
//
// Typedefs and structs
//

// Stores information about our loaded module.
struct module_info
{
	// The opened module handle
	void *handle;

	// The path of the library containing the module
	std::string filepath;

	// The function pointer for the "main" symbol
	main_funct main;

	// Default constructor - fills the fields with information queried by the OS
	module_info()
	{
	#ifdef _WIN32
		// Preamble
		constexpr DWORD buffer_size = 512;
		const HANDLE hProc = GetCurrentProcess();
		HMODULE modules[buffer_size];
		DWORD num_modules;

		// Query all currently loaded modules
		// (a possible failure condition is insufficient privileges for the process, but so far this hasn't happened)
		DWORD epm_result = EnumProcessModules(hProc, modules, buffer_size, &num_modules);
		assert(epm_result && "EnumProcessModules failed!");

		// Loop through all loaded modules to find ourselves
		for (unsigned i=0; i<num_modules; i++)
		{
			// Check for our unique tag
			auto unique_tag = GetProcAddress(modules[i], OTV_UNIQUE_MODULE_TAG_STR);
			if (unique_tag)
			{
				// Found it! Commit module handle and retrieve true module filepath
				handle = modules[i]; // we'll reuse the modules array to store the filename
				epm_result = GetModuleFileNameEx(hProc, (HMODULE)handle, (LPSTR)modules, sizeof(modules));
				assert(epm_result && "GetModuleFileNameEx failed!");

				// Store filepath
				filepath = (LPCSTR)modules;

				// The more robust linking model under Windows means we can get away with just taking a function pointer
				// to our currently in-scope global "main" symbol
				main = (main_funct)::main;

				// Done!
				return;
			}
		}
		assert(false && "INTERNAL LOGIC ERROR");
	#else
		// Iterate through all loaded modules until we find ourselves. Note that there is no failure condition. This search
		// will always succeed, since we couldn't have initiated it in the first place if we didn't exist!
		dl_iterate_phdr(module_iteration_callback, this);

		// Retrieve the "main" function address from our module symbol table
		main = (main_funct)dlsym(handle, "main");
	#endif
	}

	// The destructor.
#ifndef _WIN32
	~module_info()
	{
		// Drop the reference to our own library.
		dlclose(handle);
	}
#endif
};



//////
//
// Globals
//

// Anonymous namespace to prevent symbol collisions with other compilation units
namespace
{
	// Runtime linker information about our shared object / DLL file
	const module_info minfo;
};



//////
//
// Functions
//

#ifndef _WIN32
	// Implementation of the callback for the enumeration of modules loaded by the OS.
	int module_iteration_callback(struct dl_phdr_info* info, size_t size, void* data)
	{
		// Skip the root executable
		if (strlen(info->dlpi_name) < 1)
			return 0; // continue iteration

		// Open the module and check if it contains our unique tag
		void* handle = dlopen(info->dlpi_name, RTLD_LAZY | RTLD_NOLOAD);
		if (handle)
		{
			void* tag = dlsym(handle, OTV_UNIQUE_MODULE_TAG_STR);
			if (tag) {
				// Found it!
				auto &minfo = *(module_info*)data;
				minfo.handle = handle;
				minfo.filepath = info->dlpi_name;
				return 1; // stop enumeration
			}
			dlclose(handle);
		}
		return 0; // continue enumeration
	}
#endif

// See include file.
main_funct get_on_tube_vis_main (void) {
	return minfo.main;
}

// See include file.
const std::string& on_tube_vis_module_filepath (void) {
	return minfo.filepath;
}
