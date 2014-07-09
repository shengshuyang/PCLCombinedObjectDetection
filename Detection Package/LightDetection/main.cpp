#include "std_include.h"

int main(int argc, char *argv[])
{
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	if (filenames.size () != 1)
	{
		std::cout << "Filenames missing.\n";
		exit (-1);
	}
	std::string filename;
	filename = argv[filenames[0]];

	hough_accumulation(filename);

	return 0;
} 