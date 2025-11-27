/*Copyright(c) 2018, slugdev
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met :
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
must display the following acknowledgement :
This product includes software developed by slugdev.
4. Neither the name of the slugdev nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY SLUGDEV ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED.IN NO EVENT SHALL SLUGDEV BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#include <cstdint>
#include <vector>
#include <string>
#include "StepKernel.h"
#include <iostream>
#include <fstream>

std::vector<double> read_stl_binary(std::string file_name)
{
	/*
	according to wikipedia ...
	UINT8[80] � Header
	UINT32 � Number of triangles
	foreach triangle
	REAL32[3] � Normal vector
	REAL32[3] � Vertex 1
	REAL32[3] � Vertex 2
	REAL32[3] � Vertex 3
	UINT16 � Attribute byte count
	end	*/

	std::vector<double> nodes;
	std::ifstream file(file_name, std::ios::in | std::ios::binary);
	if (!file)
	{
		std::cout << "Failed to open binary stl file: " << file_name << "\n";
		return nodes;
	}

	char header[80];
	file.read(header, 80);

	uint32_t tris = 0;
	file.read((char*)(&tris),sizeof(uint32_t));
	nodes.resize(std::size_t(tris) * 9);
	for (std::size_t i = 0; i < tris; i++)
	{
		float_t  n[3], pts[9];
		uint16_t att;
		file.read((char*)(n), sizeof(float_t) * 3);
		file.read((char*)(pts), sizeof(float_t) * 9);
		file.read((char*)(&att), sizeof(uint16_t));
		for (int j = 0; j < 9; j++)
			nodes[i * 9 + j] = pts[j];
	}
	file.close();
	return nodes;
}

std::vector<double> read_stl_ascii(std::string file_name)
{
	std::vector<double> nodes;
	std::ifstream file;
	file.open(file_name);
	if (!file)
	{
		std::cout << "Failed to open ascii stl file: " << file_name << "\n";
		return nodes;
	}
	std::string line;
	while (std::getline(file, line))
	{
		std::istringstream iss(line);
		std::string vert;
		double  x, y, z;
		if ((iss >> vert >> x >> y >> z) && vert == "vertex")
		{
			nodes.push_back(x);
			nodes.push_back(y);
			nodes.push_back(z);
		}
	}
	file.close();
	return nodes;
}

std::vector<double> read_stl(std::string file_name)
{
	// inspired by: https://stackoverflow.com/questions/26171521/verifying-that-an-stl-file-is-ascii-or-binary
	std::ifstream file_test(file_name, std::ifstream::ate | std::ifstream::binary);
	std::vector<double> nodes;
	if (!file_test)
	{
		std::cout << "Failed to open stl file: " << file_name << "\n";
		return nodes;
	}

	auto file_size = file_test.tellg();
	file_test.close();

	// The minimum size of an empty ASCII file is 15 bytes.
	if (file_size < 15)
	{
		std::cout << "Invalid stl file: " << file_name << "\n";
		return nodes;
	}

	std::ifstream file(file_name, std::ios::in | std::ios::binary);
	if (!file)
	{
		std::cout << "Failed to open stl file: " << file_name << "\n";
		return nodes;
	}

	char first[5];
	file.read(first, 5);
	file.close();
	if (first[0] == 's' &&
		first[1] == 'o' &&
		first[2] == 'l' &&
		first[3] == 'i' &&
		first[4] == 'd')
	{
		nodes = read_stl_ascii(file_name);
	}
	else
	{
		nodes = read_stl_binary(file_name);
	}
	return nodes;
}

int main(int arv, char* argc[])
{
	double tol = 1e-6;
	bool mergeplanar = false;
	std::string out_units = "mm"; // default output units
	std::string help = "stltostp <stl_file> <step_file> [tol <value>] [units <mm|cm|m|in>]\n";

	if (arv < 3)
	{
		std::cout << "ERROR: " << help;
		return 1;
	}

	std::string input_file = argc[1];
	std::string output_file = argc[2];
	int arg_cnt = 3;
	while (arg_cnt < arv)
	{
		std::string cur_arg = argc[arg_cnt];
		if (cur_arg == "tol")
		{
			tol = std::atof(argc[arg_cnt + 1]);
			std::cout << "Minimum edge tolerance set to " << tol << "\n";
			arg_cnt++;
		}
		else if (cur_arg == "mergeplanar")
		{
			mergeplanar = true;
			std::cout << "Treating input file as a step file instead of stl...\n";
			arg_cnt++;
		}
        else if (cur_arg == "units" || cur_arg == "unit")
        {
            if (arg_cnt + 1 >= arv)
            {
                std::cout << "Missing value for units parameter\n";
                return 1;
            }
            out_units = argc[arg_cnt + 1];
            std::cout << "Output units set to " << out_units << "\n";
            arg_cnt++;
        }
		else
		{
			std::cout << "Unknown parameter " << cur_arg << "\n";
			return 1;
		}
		arg_cnt++;
	}

	std::vector<double> nodes = read_stl(input_file);
	if (nodes.size()/9 == 0)
	{
		std::cout << "No triangles found in stl file: " << input_file << "\n";
		return 1;
	}
	
	std::cout << "Read " << nodes.size() / 9 << " triangles from " << input_file << "\n";

	StepKernel se;
	int merged_edge_cnt = 0;
	se.build_tri_body(nodes,tol,merged_edge_cnt);
	se.write_step(output_file, out_units);
	std::cout << "Merged " << merged_edge_cnt << " edges\n";
	std::cout << "Exported STEP file: " << output_file << "\n";
	return 0;
}


