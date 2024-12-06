/**
 * @file
 * @brief Macro definitions required to set up linkage and exports, steering the behavior of all other header files.
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__


//////
//
// Macros
//

// --- Undocumented helper macros ------------------------
#define OTV_STRINGIFY_HELPER(arg) #arg
#define OTV_STRINGIFY(arg) OTV_STRINGIFY_HELPER(arg)
// --- [END] Undocumented helper macros ------------------

// C vs. C++ handling
#ifdef __cplusplus
	#define OTV_EXTERN_C extern "C"
#else
	#define OTV_EXTERN_C
#endif

// DLL bindings
#if defined(_WIN32) || defined(WIN32)
	#ifdef ONTUBEVIS_EXPORTS
		#define OTV_API OTV_EXTERN_C __declspec(dllexport)
	#else
		#define OTV_API OTV_EXTERN_C __declspec(dllimport)
	#endif
#else
	#define OTV_API OTV_EXTERN_C
#endif

/// @brief The symbol serving as a tag for uniquely identifying our module
#define OTV_UNIQUE_MODULE_TAG otv__unique_module_tag

/**
 * @brief
 *		A C-style string literal containing the name of the symbol serving as a tag for uniquely identifying our module.
 */
#define OTV_UNIQUE_MODULE_TAG_STR OTV_STRINGIFY(OTV_UNIQUE_MODULE_TAG)


#endif // ifndef __CONFIG_H__
