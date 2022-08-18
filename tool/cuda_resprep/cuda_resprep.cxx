
//////
//
// Includes
//

// C++ STL
#include <iostream>
#include <sstream>
#include <algorithm>
#include <filesystem>



//////
//
// Functions
//

// application entry point
int main (int argc, char** argv)
{
	if (argc > 3)
	{
		// determine paths of involved files
		std::string nvcc, infile, outfile, ptxfile;
		/* local scope */ {
			// build logical path representation
			std::filesystem::path nvcc_path(argv[1]), infile_path(argv[2]), outfile_path(argv[3]);
			nvcc_path.make_preferred(); infile_path.make_preferred(); outfile_path.make_preferred();
			std::filesystem::path ptxfile_path(outfile_path.parent_path()/infile_path.filename());
			ptxfile_path.replace_extension("ptx");

			// commit to strings using unified '/' separators
			nvcc    = nvcc_path.string();    std::replace(nvcc.begin(), nvcc.end(), '\\', '/');
			infile  = infile_path.string();  std::replace(infile.begin(), infile.end(), '\\', '/');
			outfile = outfile_path.string(); std::replace(outfile.begin(), outfile.end(), '\\', '/');
			ptxfile = ptxfile_path.string(); std::replace(ptxfile.begin(), ptxfile.end(), '\\', '/');
		}

		// build nvcc command line
		// - base invocation
		std::stringstream cmd;
		cmd << '"'<<nvcc<<"\" " << infile <<' '<< "-o="<<ptxfile;
		// - forward additional options
		for (int i=4; i<argc; i++)
			cmd << ' '<<argv[i];

		// invoke nvcc
		std::cout << "NVCC command:"<<std::endl<<"  "<<cmd.str() << std::endl;
		int exitcode = system(cmd.str().c_str());
		if (exitcode != 0)
			return 0;

		// invoke res_prep
		// - build command line
		cmd.clear(); cmd.str("");
		cmd << "res_prep " << ptxfile <<' '<< outfile;
		std::cout << std::endl<<"res_prep command:"<<std::endl<<"  "<<cmd.str() << std::endl;
		// - invoke
		exitcode = system(cmd.str().c_str());

		// done!
		return exitcode;
	}

	// too few arguments
	std::cerr << "usage: cuda_resprep  path_to_nvcc  resource_file  cuda_source_file  [...nvcc args]" << std::endl;
	return -1;
}
