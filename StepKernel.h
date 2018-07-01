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

#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <sstream>

class StepKernel
{
public:
	class Entity
	{
	public:
		Entity(std::vector<Entity*> &ent_list)
		{
			ent_list.push_back(this);
			id = ent_list.size();
		}
		virtual ~Entity()
		{}

		std::vector<std::string> tokenize(const std::string& str, const std::string& delimiters = ",")
		{
			std::vector<std::string> tokens;
			// Skip delimiters at beginning.
			std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);

			// Find first non-delimiter.
			std::string::size_type pos = str.find_first_of(delimiters, lastPos);

			while (std::string::npos != pos || std::string::npos != lastPos) 
			{
				// Found a token, add it to the vector.
				tokens.push_back(str.substr(lastPos, pos - lastPos));

				// Skip delimiters.
				lastPos = str.find_first_not_of(delimiters, pos);

				// Find next non-delimiter.
				pos = str.find_first_of(delimiters, lastPos);
			}
			return tokens;
		}

		virtual void serialize(std::ostream& stream_in) = 0;
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args) = 0;
		int id;
		std::string label;
	};

	class Direction : public Entity
	{
	public:
		Direction(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			x = 0;
			y = 0;
			z = 0;
		}
		Direction(std::vector<Entity*> &ent_list, double x_in, double y_in, double z_in): Entity(ent_list)
		{
			x = x_in;
			y = y_in;
			z = z_in;
		}

		virtual ~Direction()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = DIRECTION('"<< label<<"', (" << x << ", " << y << ", " << z << "));\n";
		}

		virtual void parse_args(std::map<int,Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of('(');
			auto en = args.find_last_of(')');
			auto arg_str = args.substr(st+1, en - st - 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::stringstream ss(arg_str);
			ss >> x >> y >> z;
		}

		double x, y, z;
	};

	class Point : public Entity
	{
	public:
		Point(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			x = 0;
			y = 0;
			z = 0;
		}
		Point(std::vector<Entity*> &ent_list, double x_in, double y_in, double z_in) : Entity(ent_list)
		{
			x = x_in;
			y = y_in;
			z = z_in;
		}

		virtual ~Point()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = CARTESIAN_POINT('" << label << "', (" << x << "," << y << "," << z << "));\n";
		}
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of('(');
			auto en = args.find_last_of(')');
			auto arg_str = args.substr(st + 1, en - st - 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::stringstream ss(arg_str);
			ss >> x >> y >> z;
		}
		double x, y, z;
	};

	class Csys3D : public Entity
	{
	public:
		Csys3D(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			dir1 = 0;
			dir2 = 0;
			point = 0;
		}

		Csys3D(std::vector<Entity*> &ent_list, Direction* dir1_in,Direction* dir2_in,Point* point_in) : Entity(ent_list)
		{
			dir1 = dir1_in;
			dir2 = dir2_in;
			point = point_in;
		}

		virtual ~Csys3D()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = AXIS2_PLACEMENT_3D('" << label << "',#" << point->id << ",#" << dir1->id << ",#" << dir2->id << ");\n";
		}

		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of(',');
			auto arg_str = args.substr(st + 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			std::stringstream ss(arg_str);
			int d1_id, d2_id, p_id;
			ss >> p_id >> d1_id >> d2_id;

			dir1 = dynamic_cast<Direction*>(ent_map[d1_id]);
			dir2 = dynamic_cast<Direction*>(ent_map[d2_id]);
			point = dynamic_cast<Point*>(ent_map[p_id]);
		}

		Direction* dir1;
		Direction* dir2;
		Point* point;
	};

	class Plane : public Entity
	{
	public:
		Plane(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			csys = 0;
		}

		Plane(std::vector<Entity*> &ent_list, Csys3D* csys_in) : Entity(ent_list)
		{
			csys = csys_in;
		}
		virtual ~Plane()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = PLANE('" << label << "',#" << csys->id <<");\n";
		}
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of(',');
			auto arg_str = args.substr(st + 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			std::stringstream ss(arg_str);
			int p_id;
			ss >> p_id;

			csys = dynamic_cast<Csys3D*>(ent_map[p_id]);
		}

		Csys3D* csys;
	};

	class OrientedEdge;

	class EdgeLoop : public Entity
	{
	public:
		EdgeLoop(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
		}
		EdgeLoop(std::vector<Entity*> &ent_list, std::vector<OrientedEdge*>& edges_in) : Entity(ent_list)
		{
			faces = edges_in;
		}
		virtual ~EdgeLoop()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			// #17 = ADVANCED_FACE('', (#18), #32, .T.);
			stream_in << "#" << id << " = EDGE_LOOP('" << label << "', (";
			for (int i = 0; i < faces.size(); i++)
			{
				stream_in << "#" << faces[i]->id;
				if (i != faces.size() - 1)
					stream_in << ",";
			}
			stream_in << "));\n";
		}
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of('(');
			auto en = args.find_last_of(')');
			auto arg_str = args.substr(st + 1, en - st - 1);
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			auto vals = tokenize(arg_str);
			for (auto v : vals)
			{
				int id = std::atoi(v.c_str());
				faces.push_back(dynamic_cast<OrientedEdge*>(ent_map[id]));
			}
			//csys = dynamic_cast<Csys3D*>(ent_map[p_id]);
		}
		std::vector<OrientedEdge*> faces;
	};

	class FaceBound : public Entity
	{
	public:
		FaceBound(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			edgeLoop = 0;
			dir = true;
		}
		FaceBound(std::vector<Entity*> &ent_list, EdgeLoop* edge_loop_in, bool dir_in) : Entity(ent_list)
		{
			edgeLoop = edge_loop_in;
			dir = dir_in;
		}
		virtual ~FaceBound()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = FACE_BOUND('" << label << "', #" << edgeLoop->id << "," << (dir?".T.":".F.") << ");\n";
		}
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of(',');
			auto arg_str = args.substr(st + 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			std::stringstream ss(arg_str);
			int p_id;
			std::string tf;
			ss >> p_id >> tf;

			edgeLoop = dynamic_cast<EdgeLoop*>(ent_map[p_id]);
			dir = (tf == ".T.");
		}
		EdgeLoop* edgeLoop;
		bool dir;
	};

	class Face : public Entity
	{
	public:
		Face(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			dir = true;
			plane = 0;
		}
		Face(std::vector<Entity*> &ent_list, std::vector<FaceBound*> face_bounds_in, Plane* plane_in, bool dir_in) : Entity(ent_list)
		{
			faceBounds = face_bounds_in;
			dir = dir_in;
			plane = plane_in;
		}
		virtual ~Face()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = ADVANCED_FACE('" << label << "', (";
			for (int i = 0; i < faceBounds.size(); i++)
			{
				stream_in << "#" << faceBounds[i]->id;
				if (i != faceBounds.size() - 1)
					stream_in << ",";
			}
			stream_in << "),#" << plane->id << "," << (dir?".T.":".F.") << ");\n";
		}
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of('(');
			auto en = args.find_last_of(')');
			auto arg_str = args.substr(st + 1, en - st - 1);
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			auto vals = tokenize(arg_str);
			for (auto v : vals)
			{
				int id = std::atoi(v.c_str());
				faceBounds.push_back(dynamic_cast<FaceBound*>(ent_map[id]));
			}
			auto remaining = args.substr(en + 1);
			std::replace(remaining.begin(), remaining.end(), '#', ' ');
			std::replace(remaining.begin(), remaining.end(), ',', ' ');
			std::stringstream ss(remaining);
			int p_id;
			std::string tf;
			ss >> p_id >> tf;

			plane = dynamic_cast<Plane*>(ent_map[p_id]);
			dir = (tf == ".T.");
		}

		std::vector<FaceBound*> faceBounds;
		bool dir;
		Plane* plane;
	};

	class Shell : public Entity
	{
	public:
		Shell(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			isOpen = true;
		}
		Shell(std::vector<Entity*> &ent_list, std::vector<Face*>& faces_in) : Entity(ent_list)
		{
			faces = faces_in;
			isOpen = true;
		}
		virtual ~Shell()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			if (isOpen)
				stream_in << "#" << id << " = OPEN_SHELL('" << label << "',(";
			else
				stream_in << "#" << id << " = CLOSED_SHELL('" << label << "',(";

			for (int i = 0; i < faces.size(); i++)
			{
				stream_in << "#" << faces[i]->id;
				if (i != faces.size() - 1)
					stream_in << ",";
			}
			stream_in << "));\n";
		}
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of('(');
			auto en = args.find_last_of(')');
			auto arg_str = args.substr(st + 1, en - st - 1);
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			auto vals = tokenize(arg_str);
			for (auto v : vals)
			{
				int id = std::atoi(v.c_str());
				faces.push_back(dynamic_cast<Face*>(ent_map[id]));
			}
		}

		std::vector<Face*> faces;
		bool isOpen;
	};

	class ShellModel : public Entity
	{
	public:
		ShellModel(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
		}
		ShellModel(std::vector<Entity*> &ent_list, std::vector<Shell*> shells_in) : Entity(ent_list)
		{
			shells = shells_in;
		}
		virtual ~ShellModel()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = SHELL_BASED_SURFACE_MODEL('" << label << "', (";
			for (int i = 0; i < shells.size(); i++)
			{
				stream_in << "#" << shells[i]->id;
				if(i != shells.size()-1)
					stream_in << ",";
			}
			stream_in << "));\n";
		}
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of('(');
			auto en = args.find_last_of(')');
			auto arg_str = args.substr(st + 1, en - st - 1);
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			auto vals = tokenize(arg_str);
			for (auto v : vals)
			{
				int id = std::atoi(v.c_str());
				shells.push_back(dynamic_cast<Shell*>(ent_map[id]));
			}
		}

		std::vector<Shell*> shells;
	};

	class ManifoldShape : public Entity
	{
	public:
		ManifoldShape(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			csys = 0;
			shellModel = 0;
		}
		ManifoldShape(std::vector<Entity*> &ent_list, Csys3D* csys_in, ShellModel* shell_model_in) : Entity(ent_list)
		{
			csys = csys_in;
			shellModel = shell_model_in;
		}
		virtual ~ManifoldShape()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = MANIFOLD_SURFACE_SHAPE_REPRESENTATION('" << label << "', (#" << csys->id << ", #" << shellModel->id << "));\n";
		}
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of('(');
			auto en = args.find_last_of(')');
			auto arg_str = args.substr(st + 1, en - st - 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			std::stringstream ss(arg_str);
			int p1_id, p2_id;
			ss >> p1_id >> p2_id;

			csys = dynamic_cast<Csys3D*>(ent_map[p1_id]);
			shellModel = dynamic_cast<ShellModel*>(ent_map[p2_id]);

			if (!csys && !shellModel)
			{
				csys = dynamic_cast<Csys3D*>(ent_map[p2_id]);
				shellModel = dynamic_cast<ShellModel*>(ent_map[p1_id]);
			}
		}

		Csys3D* csys;
		ShellModel* shellModel;
	};

	class Vertex : public Entity
	{
	public:
		Vertex(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			point = 0;
		}
		Vertex(std::vector<Entity*> &ent_list, Point* point_in) : Entity(ent_list)
		{
			point = point_in;
		}
		virtual ~Vertex()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = VERTEX_POINT('" << label << "', #" << point->id << ");\n";
		}
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of(',');
			auto arg_str = args.substr(st + 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			std::stringstream ss(arg_str);
			int p1_id;
			ss >> p1_id ;

			point = dynamic_cast<Point*>(ent_map[p1_id]);
		}

		Point* point;
	};

	class Line;

	class SurfaceCurve : public Entity
	{
	public:
		SurfaceCurve(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			line = 0;
		}
		SurfaceCurve(std::vector<Entity*> &ent_list, Line* surface_curve_in) : Entity(ent_list)
		{
			line = surface_curve_in;
		}
		virtual ~SurfaceCurve()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = SURFACE_CURVE('" << label << "', #" << line->id << ");\n";
		}
		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of(',');
			auto arg_str = args.substr(st + 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			std::stringstream ss(arg_str);
			int p1_id;
			ss >> p1_id;

			line = dynamic_cast<Line*>(ent_map[p1_id]);
		}

		Line* line;
	};

	class EdgeCurve : public Entity
	{
	public:
		EdgeCurve(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			vert1 = 0;
			vert2 = 0;
			surfCurve = 0;
			dir = true;
		}
		EdgeCurve(std::vector<Entity*> &ent_list,Vertex* vert1_in, Vertex* vert2_in, SurfaceCurve* surf_curve_in, bool dir_in) : Entity(ent_list)
		{
			vert1 = vert1_in;
			vert2 = vert2_in;
			surfCurve = surf_curve_in;
			dir = dir_in;
		}
		virtual ~EdgeCurve()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = EDGE_CURVE('', #" << vert1->id << ", #" << vert2->id << ",#" << (surfCurve ? surfCurve->id: line->id) << "," << (dir ? ".T." : ".F.") << ");\n";
		}

		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of(',');
			auto arg_str = args.substr(st + 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			std::stringstream ss(arg_str);
			int p1_id, p2_id, p3_id;
			std::string tf;
			ss >> p1_id >> p2_id >> p3_id >> tf;

			vert1 = dynamic_cast<Vertex*>(ent_map[p1_id]);
			vert2 = dynamic_cast<Vertex*>(ent_map[p2_id]);
			surfCurve = dynamic_cast<SurfaceCurve*>(ent_map[p3_id]);
			if(!surfCurve)
				line = dynamic_cast<Line*>(ent_map[p3_id]);
			dir = (tf == ".T.");
		}

		Vertex* vert1;
		Vertex* vert2;
		SurfaceCurve* surfCurve;
		Line* line;
		bool dir;
	};

	class OrientedEdge : public Entity
	{
	public:
		OrientedEdge(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			edge = 0;
			dir = 0;
		}
		OrientedEdge(std::vector<Entity*> &ent_list,EdgeCurve* edge_curve_in,bool dir_in) : Entity(ent_list)
		{
			edge = edge_curve_in;
			dir = dir_in;
		}
		virtual ~OrientedEdge()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = ORIENTED_EDGE('"<<label<<"',*,*,#" << edge->id << "," << (dir ? ".T." : ".F.") << ");\n";
		}

		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of(',');
			auto arg_str = args.substr(st + 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			std::stringstream ss(arg_str);
			int p1_id;
			std::string s1,s2,tf;
			ss >> s1 >> s2 >> p1_id >> tf;

			edge = dynamic_cast<EdgeCurve*>(ent_map[p1_id]);
			dir = (tf == ".T.");
		}

		bool dir;
		EdgeCurve* edge;
	};

	class Vector : public Entity
	{
	public:
		Vector(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			dir = 0;
			length = 0;
		}
		Vector(std::vector<Entity*> &ent_list, Direction* dir_in, double len_in) : Entity(ent_list)
		{
			dir = dir_in;
			length = len_in;
		}
		virtual ~Vector()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = VECTOR('" << label << "',#" << dir->id << "," << length << ");\n";
		}

		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of(',');
			auto arg_str = args.substr(st + 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			std::stringstream ss(arg_str);
			int p1_id;
			ss >> p1_id >> length;

			dir = dynamic_cast<Direction*>(ent_map[p1_id]);
		}

		double length;
		Direction* dir;
	};

	class Line : public Entity
	{
	public:
		Line(std::vector<Entity*> &ent_list) : Entity(ent_list)
		{
			vector = 0;
			point = 0;
		}
		Line(std::vector<Entity*> &ent_list,Point* point_in,Vector* vec_in) : Entity(ent_list)
		{
			vector = vec_in;
			point = point_in;
		}
		virtual ~Line()
		{}

		virtual void serialize(std::ostream& stream_in)
		{
			stream_in << "#" << id << " = LINE('" << label << "',#" << point->id << ", #" << vector->id << ");\n";
		}

		virtual void parse_args(std::map<int, Entity*> &ent_map, std::string args)
		{
			auto st = args.find_first_of(',');
			auto arg_str = args.substr(st + 1);
			std::replace(arg_str.begin(), arg_str.end(), ',', ' ');
			std::replace(arg_str.begin(), arg_str.end(), '#', ' ');
			std::stringstream ss(arg_str);
			int p1_id, p2_id;
			ss >> p1_id >> p2_id;

			point = dynamic_cast<Point*>(ent_map[p1_id]);
			vector = dynamic_cast<Vector*>(ent_map[p2_id]);
		}

		Point* point;
		Vector* vector;
	};

public:
	StepKernel();
	virtual ~StepKernel();

	StepKernel::EdgeCurve* create_edge_curve(StepKernel::Vertex * vert1, StepKernel::Vertex * vert2, bool dir);

	void build_tri_body(std::vector<double> tris, double tol);
	void get_edge_from_map(
		double  p0[3],
		double  p1[3],
		std::map<std::tuple<double, double, double, double, double, double>, StepKernel::EdgeCurve *> &edge_map,
		StepKernel::Vertex * vert1,
		StepKernel::Vertex * vert2,
		EdgeCurve *& edge_curve,
		bool &edge_dir);
	void write_step(std::string file_name);
	std::string StepKernel::read_line(std::ifstream &stp_file, bool skip_all_space);
	void read_step(std::string file_name);
	std::vector<Entity*> entities;
};