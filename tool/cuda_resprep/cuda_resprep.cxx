
//////
//
// Includes
//

// C++ STL
#include <iostream>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <regex>



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
		std::stringstream cmd_part1, cmd_part2;
		cmd_part1 << '"'<<infile<<"\" " << "-o=\""<<ptxfile<<'"';
		// - forward additional options
		for (int i=4; i<argc; i++)
			// TODO: support other NVCC options that take paths as input
			if (std::strlen(argv[i]) > 2 && argv[i][0]=='-' && argv[i][1]=='I')
			{
				std::string path(argv[i] + 2);
				std::replace(path.begin(), path.end(), '\\', '/');
				cmd_part2 << " -I\""<<path<<"\"";
			}
			else
				cmd_part2 << ' '<<argv[i];

		// invoke nvcc
		std::cout << "NVCC command:"<<std::endl<<"  " << nvcc <<' '<< cmd_part1.str() << cmd_part2.str() << std::endl;
		// - compose final command with quotes escaped
		std::string escaped_args_str = std::regex_replace(cmd_part2.str(), std::regex("\""), "\\\"");
		std::stringstream cmd_final;
		cmd_final << nvcc <<' '<< cmd_part1.str() << cmd_part2.str();
		// - dispatch NVCC call
		int exitcode = system(cmd_final.str().c_str());
		if (exitcode != 0)
			return exitcode;

		// invoke res_prep
		// - build command line
		cmd_final.clear(); cmd_final.str("");
		cmd_final << "res_prep " << "\""<<ptxfile<<"\" \""<<outfile<<"\"";
		std::cout << "res_prep command:"<<std::endl<<"  "<< cmd_final.str() << std::endl<<std::endl;
		// - invoke
		exitcode = system(cmd_final.str().c_str());

		// done!
		return exitcode;
	}

	// too few arguments
	std::cerr << "usage: cuda_resprep  path_to_nvcc  cuda_source_file  out_resource_cppfile  [...nvcc args]" << std::endl;
	return -1;
}
