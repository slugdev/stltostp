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

#include <vector>
#include <string>
#include "StepKernel.h"
#include <iostream>
#include <fstream>

std::vector<double> read_stl_ascii(std::string file_name)
{
	std::vector<double> nodes;
	std::ifstream file;
	file.open(file_name);
	if (!file)
	{
		std::cout << "Failed to open stl file: " << file_name << "\n";
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

int main(int arv, char* argc[])
{
	double tol = 1e-6;
	std::string help = "stltostp <stl_file> <step_file> [tol <value>]\n";

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
		else
		{
			std::cout << "Unknown parameter " << cur_arg << "\n";
			return 1;
		}
		arg_cnt++;
	}

	std::vector<double> nodes = read_stl_ascii(input_file);
	if (nodes.size()/3 == 0)
	{
		std::cout << "No triangles found in stl file: " << input_file << "\n";
		return 1;
	}
	
	std::cout << "Read " << nodes.size() / 3 << " trianges from " << input_file << "\n";

	StepKernel se;
	se.build_tri_body(nodes,tol);
	se.write_step(output_file);
	std::cout << "Exported STEP file: " << output_file << "\n";
	return 0;
}


