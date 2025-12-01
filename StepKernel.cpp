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

#include "StepKernel.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <sstream>
#include <map>
#include <iomanip> // put_time
#include <cctype>
StepKernel::StepKernel()
{

}

StepKernel::~StepKernel()
{
}

StepKernel::EdgeCurve* StepKernel::create_edge_curve(StepKernel::Vertex * vert1, StepKernel::Vertex * vert2,bool dir)
{
	// curve 1
	auto line_point1 = new Point(entities, vert1->point->x, vert1->point->y, vert1->point->z);
	double vx = vert2->point->x - vert1->point->x;
	double vy = vert2->point->y - vert1->point->y;
	double vz = vert2->point->z - vert1->point->z;
	double dist = sqrt(vx * vx + vy * vy + vz * vz);
	vx = vx / dist;
	vy = vy / dist;
	vz = vz / dist;

	auto line_dir1 = new Direction(entities, vx, vy, vz);
	auto line_vector1 = new Vector(entities, line_dir1, 1.0);
	auto line1 = new Line(entities, line_point1, line_vector1);
	auto surf_curve1 = new SurfaceCurve(entities, line1);
	return new EdgeCurve(entities, vert1, vert2, surf_curve1, dir);
}

void StepKernel::build_tri_body(std::vector<double> tris,double tol, int &merged_edge_cnt)
{
	auto point = new Point(entities, 0.0, 0.0, 0.0);
	auto dir_1 = new Direction(entities, 0.0, 0.0, 1.0);
	auto dir_2 = new Direction(entities, 1.0, 0.0, 0.0);

	auto base_csys = new Csys3D(entities, dir_1, dir_2, point);
	std::vector<Face*> faces;
	std::map<std::tuple<double, double, double, double, double, double>, EdgeCurve*> edge_map;
	for (std::size_t i = 0; i < tris.size() / 9; i++)
	{
		double p0[3] = { tris[i * 9 + 0],tris[i * 9 + 1] ,tris[i * 9 + 2] };
		double p1[3] = { tris[i * 9 + 3],tris[i * 9 + 4] ,tris[i * 9 + 5] };
		double p2[3] = { tris[i * 9 + 6],tris[i * 9 + 7] ,tris[i * 9 + 8] };

		double d0[3] = { 1,0,0 };
		d0[0] = p1[0] - p0[0];
		d0[1] = p1[1] - p0[1];
		d0[2] = p1[2] - p0[2];
		double dist0 = sqrt(d0[0] * d0[0] + d0[1] * d0[1] + d0[2] * d0[2]);
		if (dist0 < tol)
			continue;
		d0[0] = d0[0] / dist0;
		d0[1] = d0[1] / dist0;
		d0[2] = d0[2] / dist0;

		double d1[3] = { 1,0,0 };
		d1[0] = p2[0] - p0[0];
		d1[1] = p2[1] - p0[1];
		d1[2] = p2[2] - p0[2];
		double dist1 = sqrt(d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2]);
		if (dist1 < tol)
			continue;
		d1[0] = d1[0] / dist1;
		d1[1] = d1[1] / dist1;
		d1[2] = d1[2] / dist1;

		// now cross
		// cross to get the thrid direction for the beam csys
		double d2[3] = { d0[1] * d1[2] - d0[2] * d1[1], d0[2] * d1[0] - d0[0] * d1[2], d0[0] * d1[1] - d0[1] * d1[0] };
		double dist2 = sqrt(d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2]);
		if (dist2 < tol)
			continue;
		d2[0] = d2[0] / dist2;
		d2[1] = d2[1] / dist2;
		d2[2] = d2[2] / dist2;

		// correct the m direction
		double d1_cor[3] = { d2[1] * d0[2] - d2[2] * d0[1], d2[2] * d0[0] - d2[0] * d0[2], d2[0] * d0[1] - d2[1] * d0[0] };
		double d1_cor_len = sqrt(d1_cor[0] * d1_cor[0] + d1_cor[1] * d1_cor[1] + d1_cor[2] * d1_cor[2]);
		d1[0] = d1_cor[0] / d1_cor_len;
		d1[1] = d1_cor[1] / d1_cor_len;
		d1[2] = d1_cor[2] / d1_cor_len;


		// build the face
		// the 3 vertex locations
		auto point1 = new Point(entities, p0[0], p0[1], p0[2]);
		auto vert1 = new Vertex(entities, point1);

		auto point2 = new Point(entities, p1[0], p1[1], p1[2]);
		auto vert2 = new Vertex(entities, point2);

		auto point3 = new Point(entities, p2[0], p2[1], p2[2]);
		auto vert3 = new Vertex(entities, point3);


		EdgeCurve* edge_curve1 = 0;
		bool edge1_dir = true;
		get_edge_from_map(p0, p1, edge_map, vert1, vert2, edge_curve1, edge1_dir, merged_edge_cnt);

		EdgeCurve* edge_curve2 = 0;
		bool edge2_dir = true;
		get_edge_from_map(p1, p2, edge_map, vert2, vert3, edge_curve2, edge2_dir, merged_edge_cnt);

		EdgeCurve* edge_curve3 = 0;
		bool edge3_dir = true;
		get_edge_from_map(p2, p0, edge_map, vert3, vert1, edge_curve3, edge3_dir, merged_edge_cnt);

		std::vector<OrientedEdge*> oriented_edges;
		oriented_edges.push_back(new OrientedEdge(entities, edge_curve1, edge1_dir));
		oriented_edges.push_back(new OrientedEdge(entities, edge_curve2, edge2_dir));
		oriented_edges.push_back(new OrientedEdge(entities, edge_curve3, edge3_dir));

		// create the plane
		auto plane_point = new Point(entities, p0[0], p0[1], p0[2]);
		auto plane_dir_1 = new Direction(entities, d2[0], d2[1], d2[2]);
		auto plane_dir_2 = new Direction(entities, d0[0], d0[1], d0[2]);
		auto plane_csys = new Csys3D(entities, plane_dir_1, plane_dir_2, plane_point);
		auto plane = new Plane(entities, plane_csys);

		// build the faces

		auto edge_loop = new EdgeLoop(entities, oriented_edges);
		std::vector<FaceBound*> face_bounds;
		face_bounds.push_back(new FaceBound(entities, edge_loop, true));
		faces.push_back(new Face(entities, face_bounds, plane, true));
	}

	// build the model
	auto open_shell = new Shell(entities, faces);
	std::vector<Shell*> shells;
	shells.push_back(open_shell);
	auto shell_model = new ShellModel(entities, shells);
	auto manifold_shape = new ManifoldShape(entities, base_csys, shell_model);
}

void StepKernel::get_edge_from_map(
	double  p0[3],
	double  p1[3],
	std::map<std::tuple<double, double, double, double, double, double>, StepKernel::EdgeCurve *> &edge_map,
	StepKernel::Vertex * vert1,
	StepKernel::Vertex * vert2,
	EdgeCurve *& edge_curve,
	bool &edge_dir,
	int &merge_cnt)
{
	edge_curve = 0;
	edge_dir = true;
	auto edge_tuple1_f = std::make_tuple(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);
	auto edge_tuple1_r = std::make_tuple(p1[0], p1[1], p1[2], p0[0], p0[1], p0[2]);
	if (edge_map.count(edge_tuple1_f))
	{
		edge_curve = edge_map[edge_tuple1_f];
		edge_dir = true;
		merge_cnt++;
	}
	else if (edge_map.count(edge_tuple1_r))
	{
		edge_curve = edge_map[edge_tuple1_r];
		edge_dir = false;
		merge_cnt++;
	}
	if (!edge_curve)
	{
		edge_curve = create_edge_curve(vert1, vert2, true);
		edge_map[edge_tuple1_f] = edge_curve;
	}
}

void StepKernel::write_step(std::string file_name, const std::string &unit, const std::string &schema)
{
	std::time_t tt = std ::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	struct std::tm * ptm = std::localtime(&tt);
	std::stringstream iso_time;
	iso_time << std::put_time(ptm, "%FT%T");

	std::ofstream stp_file;
	stp_file.open(file_name);
	if (!stp_file)
		return;

	// Set precision for floating point numbers to preserve accuracy
	stp_file << std::fixed << std::setprecision(15);

	// Normalize and map unit token to STEP LENGTH_UNIT name and scale (metres per unit)
	std::string u = unit;
	for (auto &c : u) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	std::string step_unit_label = "MILLIMETRE";
	double unit_scale = 0.001; // metres per unit
	if (u == "mm" || u == "millimetre" || u == "millimeter")
	{
		step_unit_label = "MILLIMETRE";
		unit_scale = 0.001;
	}
	else if (u == "cm" || u == "centimetre" || u == "centimeter")
	{
		step_unit_label = "CENTIMETRE";
		unit_scale = 0.01;
	}
	else if (u == "m" || u == "metre" || u == "meter")
	{
		step_unit_label = "METRE";
		unit_scale = 1.0;
	}
	else if (u == "in" || u == "inch" || u == "inches")
	{
		step_unit_label = "INCH";
		unit_scale = 0.0254;
	}
	else
	{
		// Unknown unit token; default to millimetre
		step_unit_label = "MILLIMETRE";
		unit_scale = 0.001;
	}

	std::string author = "slugdev";
	std::string org = "org";

	// Header section - ISO 10303-21
	stp_file << "ISO-10303-21;\n";
	stp_file << "HEADER;\n";
	stp_file << "FILE_DESCRIPTION(('Configuration controlled 3D design of mechanical parts and assemblies'),'2;1');\n";
	stp_file << "FILE_NAME('" << file_name << "','" << iso_time.str() << "',('" << author << "'),('" << org << "'),'StepKernel by stltostp','stltostp v1.0.1',' ');\n";
	// Default to AP203 unless schema token requests 214
	std::string s = schema;
	for (auto &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	if (s == "214" || s == "ap214")
		stp_file << "FILE_SCHEMA(('AP214IS'),'3');\n";
	else
		stp_file << "FILE_SCHEMA(('AP203'),'2');\n";
	stp_file << "ENDSEC;\n";

	// Data section
	stp_file << "DATA;\n";

	// Get the last entity ID to generate new wrapper entity IDs
	int next_id = static_cast<int>(entities.size()) + 1;

	// Serialize all geometric/topological entities first
	for (auto e : entities)
		e->serialize(stp_file);

	// Get reference to the last created entity (should be ManifoldShape)
	// Assuming the last entity is the top-level ManifoldShape created in build_tri_body
	int manifold_shape_id = static_cast<int>(entities.size());

	// Emit context/wrapper entities. For AP214, emit fuller set; for AP203 keep minimal.
	// Unit assignment (uses the next LENGTH_UNIT entity)
	stp_file << "#" << next_id << " = UNIT_ASSIGNMENT((#" << (next_id + 1) << "));\n";
	next_id++;

	// Length unit using chosen unit label and scale (metres per unit)
	stp_file << "#" << next_id << " = LENGTH_UNIT('" << step_unit_label << "'," << unit_scale << ");\n";
	next_id++;

	std::string sfull = schema;
	for (auto &c : sfull) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	if (sfull == "214" || sfull == "ap214")
	{
		// Geometric representation context (3D)
		stp_file << "#" << next_id << " = GEOMETRIC_REPRESENTATION_CONTEXT('2D',#" << (next_id + 1) << ",#" << (next_id + 2) << ");\n";
		next_id++;

		// Parametric representation context
		stp_file << "#" << next_id << " = PARAMETRIC_REPRESENTATION_CONTEXT('3D',#" << (next_id + 1) << ");\n";
		next_id++;

		// Representation context (for general use)
		stp_file << "#" << next_id << " = REPRESENTATION_CONTEXT('3D','3D Space');\n";
		next_id++;

		// Axis placement 3D (global coordinate system) for product context
		int placement_id = next_id;
		stp_file << "#" << next_id << " = AXIS2_PLACEMENT_3D('Global Origin',#" << (next_id + 1) << ",#" << (next_id + 2) << ",#" << (next_id + 3) << ");\n";
		next_id++;

		// Origin point (0,0,0) for global placement
		stp_file << "#" << next_id << " = CARTESIAN_POINT('Origin',(0.0,0.0,0.0));\n";
		next_id++;

		// Z direction
		stp_file << "#" << next_id << " = DIRECTION('Z',(0.0,0.0,1.0));\n";
		next_id++;

		// X direction
		stp_file << "#" << next_id << " = DIRECTION('X',(1.0,0.0,0.0));\n";
		next_id++;

		// Product definition shape (references the manifold)
		int prod_def_shape_id = next_id;
		stp_file << "#" << next_id << " = PRODUCT_DEFINITION_SHAPE('Converted Mesh',' ',#" << (next_id + 1) << ");\n";
		next_id++;

		// Shape definition representation (holds the manifold shape and context)
		stp_file << "#" << next_id << " = SHAPE_DEFINITION_REPRESENTATION(#" << prod_def_shape_id << ",#" << (next_id + 1) << ");\n";
		next_id++;

		// Representation with manifold and context references
		stp_file << "#" << next_id << " = REPRESENTATION('Mesh Representation',(#" << manifold_shape_id << "),#" << (next_id - 5) << ");\n";
		next_id++;

		// Product type
		int product_id = next_id;
		stp_file << "#" << next_id << " = PRODUCT('Converted Mesh Part','Converted Mesh Part','STL Converted Part',( ));\n";
		next_id++;

		// Product definition context (design context)
		stp_file << "#" << next_id << " = PRODUCT_DEFINITION_CONTEXT('3D Mechanical Parts',#" << (next_id + 1) << "'part definition');\n";
		next_id++;

		// Application context
		stp_file << "#" << next_id << " = APPLICATION_CONTEXT('3D mechanical design');\n";
		next_id++;

		// Product definition (defines the product at a specific lifecycle state)
		stp_file << "#" << next_id << " = PRODUCT_DEFINITION('design','',#" << product_id << ",#" << (next_id - 2) << ");\n";
		next_id++;

		// Mechanical design geometric presentation area
		stp_file << "#" << next_id << " = PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE(' ','',#" << (next_id - 1) << ",.MADE_FROM.);\n";
	}
	else
	{
		// For AP203 we keep the file minimal: units are written above.
		// Additional AP203-specific wrapper entities can be added if required.
	}

	stp_file << "ENDSEC;\n";
	stp_file << "END-ISO-10303-21;\n";
	stp_file.close();
}

std::string StepKernel::read_line(std::ifstream &stp_file, bool skip_all_space)
{
	std::string line_str;
	bool leading_space = true;
	while (stp_file)
	{
		char get_char = ' ';
		stp_file.get(get_char);
		if (get_char == ';')
			break;

		if (get_char == '\n' || get_char == '\r' || get_char == '\t')
			continue;

		if (leading_space && (get_char == ' ' || get_char == '\t'))
			continue;
		if (!skip_all_space)
			leading_space = false;
		line_str.push_back(get_char);
	}
	return line_str;
}

void StepKernel::read_step(std::string file_name)
{
	std::ifstream stp_file;
	stp_file.open(file_name);
	if (!stp_file)
		return;

	// read the first line to get the iso stuff
	std::string iso_line = read_line(stp_file, true);

	bool data_section = false;
	std::vector<Entity*> ents;
	std::map<int,Entity*> ent_map;
	std::vector<std::string> args;
	while (stp_file)
	{
		std::string cur_str = read_line(stp_file, false);
		if (cur_str == "DATA")
		{
			data_section = true;
			continue;
		}
		if (!data_section)
			continue;

		if (cur_str == "ENDSEC")
		{
			data_section = false;
			break;
		}
		// parse the id
		int id = -1;
		if (cur_str.size() > 0 && cur_str[0] == '#' && cur_str.find('='))
		{
			auto equal_pos = cur_str.find('=');
			auto paren_pos = cur_str.find('(');
			auto id_str = cur_str.substr(1, equal_pos - 1);
			id = std::atoi(id_str.c_str());
			auto func_start = cur_str.find_first_not_of("\t ", equal_pos+1);
			auto func_end = cur_str.find_first_of("\t (", func_start + 1);
			auto func_name = cur_str.substr(func_start, func_end - func_start);

			// now parse the args
			auto arg_end = cur_str.find_last_of(')');
			auto arg_str = cur_str.substr(func_end + 1, arg_end - func_end - 1);
			Entity* ent = 0;
			if (func_name == "CARTESIAN_POINT")
				ent = new Point(entities);
			else if (func_name == "DIRECTION")
				ent = new Direction(entities);
			else if (func_name == "AXIS2_PLACEMENT_3D")
				ent = new Csys3D(entities);
			else if (func_name == "PLANE")
				ent = new Plane(entities);
			else if (func_name == "EDGE_LOOP")
				ent = new EdgeLoop(entities);
			else if (func_name == "FACE_BOUND")
				ent = new FaceBound(entities);
			else if (func_name == "FACE_OUTER_BOUND")
				ent = new FaceBound(entities);
			else if (func_name == "ADVANCED_FACE")
				ent = new Face(entities);
			else if (func_name == "FACE_SURFACE")
				ent = new Face(entities);
			else if (func_name == "OPEN_SHELL")
				ent = new Shell(entities);
			else if (func_name == "CLOSED_SHELL")
				ent = new Shell(entities);
			else if (func_name == "SHELL_BASED_SURFACE_MODEL")
				ent = new ShellModel(entities);
			else if (func_name == "MANIFOLD_SURFACE_SHAPE_REPRESENTATION")
				ent = new ManifoldShape(entities);
			else if (func_name == "VERTEX_POINT")
				ent = new Vertex(entities);
			else if (func_name == "SURFACE_CURVE")
				ent = new SurfaceCurve(entities);
			else if (func_name == "EDGE_CURVE")
				ent = new EdgeCurve(entities);
			else if (func_name == "ORIENTED_EDGE")
				ent = new OrientedEdge(entities);
			else if (func_name == "VECTOR")
				 ent = new Vector(entities);
			else if (func_name == "LINE")
				 ent = new Line(entities);

			if (ent)
			{
				ent->id = id;
				ent_map[id] = ent;
				ents.push_back(ent);
				args.push_back(arg_str);
			}
		}
		std::cout << cur_str << "\n";
	}
	// processes all the arguments
	for (int i = 0; i < ents.size(); i++)
	{
		ents[i]->parse_args(ent_map,args[i]);
	}
	stp_file.close();
}
