/*
	File:
		Point3dUtils.hpp
	Description:
		Utility functions to manipulate Point3d data, including
		* Point3d vector and Eigen::MatrixX3d converter
		* read/write vertex list from/to PLY file
	Copyright:
		Orisol Asia Ltd.
*/
#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <type_traits>
#include <typeinfo>
#include <Eigen/core>

struct Point3d
{
	double x;
	double y;
	double z;
};

struct Point3f
{
	float x;
	float y;
	float z;
};

struct Normal3d
{
	double nx;
	double ny;
	double nz;
};

struct Normal3f
{
	float nx;
	float ny;
	float nz;
};

struct Angle3d
{
	double rx;
	double ry;
	double rz;
};

struct Angle3f
{
	float rx;
	float ry;
	float rz;
};

struct Vertex6d
{
	Point3d point;
	union {
		Normal3d normal;
		Angle3d angle;
	};
};

struct Vertex6f
{
	Point3f point;
	union {
		Normal3f normal;
		Angle3f angle;
	};
};

// Metadata specific for Orisol's use of point cloud
// besides scan raw data, it is not necessary to be in the form of grid
struct CloudMeta
{
	uint32_t vertex_size;
	uint32_t grid_width;
	uint32_t grid_length;
	double x_resolution;
	double y_resolution;
	double z_resolution;
	std::string datetime;
	std::string euler_angle;
};

class CloudConvert
{
public:
	// convert from Point3f vector to MatrixX3f
	static inline Eigen::MatrixX3f toMatrixX3f(const std::vector<Point3f> & pt3fvector)
	{
		Eigen::Map<Eigen::Matrix<float, -1, 3, Eigen::RowMajor>> vec2mat((float*)pt3fvector.data(), pt3fvector.size(), 3);
		return vec2mat;
	}

	// convert from Point3d vector to MatrixX3d
	static inline Eigen::MatrixX3d toMatrixX3d(const std::vector<Point3d> & pt3dvector)
	{
		Eigen::Map<Eigen::Matrix<double, -1, 3, Eigen::RowMajor>> vec2mat((double *)pt3dvector.data(), pt3dvector.size(), 3);
		return vec2mat;
	}

	// convert from MatrixX3f to Point3f vector
	static inline std::vector<Point3f> toPoint3fVector(const Eigen::MatrixX3f & mat3f)
	{
		std::vector<Point3f> mat2vec(mat3f.rows());
		if (mat3f.IsRowMajor)
		{
			std::copy(mat3f.data(), mat3f.data() + mat3f.size(), stdext::make_checked_array_iterator((float *)mat2vec.data(), mat3f.size()));
		}
		else
		{
			Eigen::Matrix<float, -1, 3, Eigen::RowMajor> mat3f_rowmajor = mat3f;
			std::copy(mat3f_rowmajor.data(), mat3f_rowmajor.data() + mat3f_rowmajor.size(), stdext::make_checked_array_iterator((float *)mat2vec.data(), mat3f_rowmajor.size()));
		}
		return mat2vec;
	}

	// convert from MatrixX3d to Point3d vector
	static inline std::vector<Point3d> toPoint3dVector(const Eigen::MatrixX3d & mat3d)
	{
		std::vector<Point3d> mat2vec(mat3d.rows());
		if (mat3d.IsRowMajor)
		{
			std::copy(mat3d.data(), mat3d.data() + mat3d.size(), stdext::make_checked_array_iterator((double *)mat2vec.data(), mat3d.size()));
		}
		else
		{
			Eigen::Matrix<double, -1, 3, Eigen::RowMajor> mat3d_rowmajor = mat3d;
			std::copy(mat3d_rowmajor.data(), mat3d_rowmajor.data() + mat3d_rowmajor.size(), stdext::make_checked_array_iterator((double *)mat2vec.data(), mat3d_rowmajor.size()));
		}
		return mat2vec;
	}

	static inline std::vector<Vertex6d> mergeToVertex6d(const std::vector<Point3d> & pt3dvector, const std::vector<Normal3d> & n3dvector)
	{
		std::vector<Vertex6d> vtx6dvector(pt3dvector.size() <= n3dvector.size() ? pt3dvector.size() : n3dvector.size());
		if (pt3dvector.size() <= n3dvector.size())
			std::transform(pt3dvector.cbegin(), pt3dvector.cend(), n3dvector.cbegin(), vtx6dvector.begin(),
				[] (Point3d pt3d, Normal3d n3d) {
					return Vertex6d{ pt3d, n3d };
				});
		else
			std::transform(n3dvector.cbegin(), n3dvector.cend(), pt3dvector.cbegin(), vtx6dvector.begin(),
				[] (Normal3d n3d, Point3d pt3d) {
					return Vertex6d{ pt3d, n3d };
				});
		return vtx6dvector;
	}

	static inline std::vector<Vertex6d> mergeToVertex6d(const std::vector<Point3d> & pt3dvector, const std::vector<Angle3d> & a3dvector)
	{
		std::vector<Vertex6d> vtx6dvector(pt3dvector.size() <= a3dvector.size() ? pt3dvector.size() : a3dvector.size());
		if (pt3dvector.size() <= a3dvector.size())
			std::transform(pt3dvector.cbegin(), pt3dvector.cend(), a3dvector.cbegin(), vtx6dvector.begin(),
				[] (Point3d pt3d, Angle3d a3d) {
					Vertex6d vtx6d;
					vtx6d.point = pt3d;
					vtx6d.angle = a3d;
					return vtx6d;
				});
		else
			std::transform(a3dvector.cbegin(), a3dvector.cend(), pt3dvector.cbegin(), vtx6dvector.begin(),
				[] (Angle3d a3d, Point3d pt3d) {
					Vertex6d vtx6d;
					vtx6d.point = pt3d;
					vtx6d.angle = a3d;
					return vtx6d;
				});
		return vtx6dvector;
	}

	static inline std::vector<Vertex6f> mergeToVertex6f(const std::vector<Point3f> & pt3fvector, const std::vector<Normal3f> & n3fvector)
	{
		std::vector<Vertex6f> vtx6fvector(pt3fvector.size() <= n3fvector.size() ? pt3fvector.size() : n3fvector.size());
		if (pt3fvector.size() <= n3fvector.size())
			std::transform(pt3fvector.cbegin(), pt3fvector.cend(), n3fvector.cbegin(), vtx6fvector.begin(),
				[] (Point3f pt3f, Normal3f n3f) {
					return Vertex6f{ pt3f, n3f };
				});
		else
			std::transform(n3fvector.cbegin(), n3fvector.cend(), pt3fvector.cbegin(), vtx6fvector.begin(),
				[] (Normal3f n3f, Point3f pt3f) {
					return Vertex6f{ pt3f, n3f };
				});
		return vtx6fvector;
	}

	static inline std::vector<Vertex6f> mergeToVertex6f(const std::vector<Point3f> & pt3fvector, const std::vector<Angle3f> & a3fvector)
	{
		std::vector<Vertex6f> vtx6fvector(pt3fvector.size() <= a3fvector.size() ? pt3fvector.size() : a3fvector.size());
		if (pt3fvector.size() <= a3fvector.size())
			std::transform(pt3fvector.cbegin(), pt3fvector.cend(), a3fvector.cbegin(), vtx6fvector.begin(),
				[] (Point3f pt3f, Angle3f a3f) {
					Vertex6f vtx6f;
					vtx6f.point = pt3f;
					vtx6f.angle = a3f;
					return vtx6f;
				});
		else
			std::transform(a3fvector.cbegin(), a3fvector.cend(), pt3fvector.cbegin(), vtx6fvector.begin(),
				[] (Angle3f a3f, Point3f pt3f) {
					Vertex6f vtx6f;
					vtx6f.point = pt3f;
					vtx6f.angle = a3f;
					return vtx6f;
				});
		return vtx6fvector;
	}

	static inline void splitFromVertex6d(const std::vector<Vertex6d> & vtx6dvector, std::vector<Point3d> & pt3dvector, std::vector<Normal3d> & n3dvector)
	{
		if (!pt3dvector.empty())
			pt3dvector.clear();
		if (!n3dvector.empty())
			n3dvector.clear();
		for (const auto & vertex : vtx6dvector)
		{
			pt3dvector.push_back(vertex.point);
			n3dvector.push_back(vertex.normal);
		}
	}

	static inline void splitFromVertex6d(const std::vector<Vertex6d> & vtx6dvector, std::vector<Point3d> & pt3dvector, std::vector<Angle3d> & a3dvector)
	{
		if (!pt3dvector.empty())
			pt3dvector.clear();
		if (!a3dvector.empty())
			a3dvector.clear();
		for (const auto & vertex : vtx6dvector)
		{
			pt3dvector.push_back(vertex.point);
			a3dvector.push_back(vertex.angle);
		}
	}

	static inline void splitFromVertex6f(const std::vector<Vertex6f> & vtx6fvector, std::vector<Point3f> & pt3fvector, std::vector<Normal3f> & n3fvector)
	{
		if (!pt3fvector.empty())
			pt3fvector.clear();
		if (!n3fvector.empty())
			n3fvector.clear();
		for (const auto & vertex : vtx6fvector)
		{
			pt3fvector.push_back(vertex.point);
			n3fvector.push_back(vertex.normal);
		}
	}

	static inline void splitFromVertex6f(const std::vector<Vertex6f> & vtx6fvector, std::vector<Point3f> & pt3fvector, std::vector<Angle3f> & a3fvector)
	{
		if (!pt3fvector.empty())
			pt3fvector.clear();
		if (!a3fvector.empty())
			a3fvector.clear();
		for (const auto & vertex : vtx6fvector)
		{
			pt3fvector.push_back(vertex.point);
			a3fvector.push_back(vertex.angle);
		}
	}
};

// NOTE: Only very restricted subset of PLY file is implemented, e.g.
//       Vertax must be the first element, and accept only 3 combinations of properties:
//       (1). three properties x, y, z in float or double
//       (2). six properties x, y, z, nx, ny, nz in float or double
//       (3). six properties x, y, z, rx, ry, rz in float or double, and with euler angle type
//       All other elements in file are ignored
class PlyFile
{
private:
	enum class Format : uint8_t
	{
		binary_big_endian,
		binary_little_endian,
		ascii
	};

	enum class DataType : uint8_t
	{
		Invalid,
		Char,
		Uchar,
		Short,
		Ushort,
		Int,
		Uint,
		Float,
		Double
	};

	// TODO: to support multiple element & property, need additional structure to store their information
	struct Header
	{
		Format file_format;
		uint32_t vertex_size;
		uint32_t grid_width;
		uint32_t grid_length;
		double x_resolution;
		double y_resolution;
		double z_resolution;
		std::string datetime;
		std::string euler_angle;
		uint32_t property_count;
		DataType property_datatype;
		std::streampos vertex_position;
	};

	//------------------------------------------------------------------------------------
	// parse the PLY header from input stream
	// return false if not an expected PLY file header format
	//------------------------------------------------------------------------------------
	static bool parseHeader(std::ifstream & input, Header & header)
	{
		// prepare the tool and buffer we need
		std::string aline;
		std::istringstream ssin;
		std::string keyword, token1, token2;

		// get the first line
		if (!std::getline(input, aline))
			return false;
		// the 1st keyword must be the magic number "ply"
		ssin.str(aline);
		ssin >> keyword;
		if (keyword != "ply" && keyword != "PLY")
			return false;

		// get the second line
		if (!std::getline(input, aline))
			return false;
		// the 2nd keyword must be the file format indicator
		ssin.str(aline);
		ssin.seekg(0, std::ios::beg);
		ssin >> keyword >> token1 >> token2;
		if (keyword != "format")
			return false;
		if (token1 == "binary_little_endian")
			header.file_format = Format::binary_little_endian;
		else if (token1 == "ascii")
			header.file_format = Format::ascii;
		else
			header.file_format = Format::binary_big_endian;
		// (token2 == "1.0"), does not matter

		while (std::getline(input, aline))
		{
			ssin.str(aline);
			ssin.seekg(0, std::ios::beg);
			ssin >> keyword >> token1 >> token2;
			// vertex element is all we need
			if (keyword == "element" && token1 == "vertex")
			{
				// NOTE: exception may throw if string failed to convert to number
				header.vertex_size = std::stoul(token2);
				// keep reading for the properties of vertex element
				header.property_count = 0;
				while (std::getline(input, aline))
				{
					ssin.str(aline);
					ssin.seekg(0, std::ios::beg);
					ssin >> keyword;
					if (keyword == "comment")
					{
						ssin >> token1;
						// "dimension" contains our own private hint for grid dimension
						if (token1 == "dimension")
						{
							// NOTE: exception may throw if string failed to convert to number
							ssin >> token2;
							header.grid_width = std::stoul(token2);
							ssin >> token2;
							header.grid_length = std::stoul(token2);
						}
						else if (token1 == "resolution")
						{
							ssin >> token2;
							header.x_resolution = std::stod(token2);
							ssin >> token2;
							header.y_resolution = std::stod(token2);
							ssin >> token2;
							header.z_resolution = std::stod(token2);
						}
						else if (token1 == "datetime")
						{
							ssin >> token2;
							header.datetime = token2;
						}
						else if (token1 == "euler")
						{
							ssin >> token2;
							header.euler_angle = token2;
						}
					}
					else if (keyword == "property")
					{
						header.property_count += 1;
						// data type and the order of property
						ssin >> token1 >> token2;
						if (token1 == "char")
							header.property_datatype = DataType::Char;
						else if (token1 == "uchar")
							header.property_datatype = DataType::Uchar;
						else if (token1 == "short")
							header.property_datatype = DataType::Short;
						else if (token1 == "ushort")
							header.property_datatype = DataType::Ushort;
						else if (token1 == "int")
							header.property_datatype = DataType::Int;
						else if (token1 == "uint")
							header.property_datatype = DataType::Uint;
						else if (token1 == "float")
							header.property_datatype = DataType::Float;
						else if (token1 == "double")
							header.property_datatype = DataType::Double;
						else
							header.property_datatype = DataType::Invalid;
					}
					else
						break;
				}
			}

			if (keyword == "end_header")
			{
				header.vertex_position = input.tellg();
				break;
			}
		}
		return true;
	}

	//------------------------------------------------------------------------------------
	// helper function to read continuous binary data into a vector.
	// the source and target data types can be different but must be convertible
	//------------------------------------------------------------------------------------
	template <typename sourceType, typename targetType>
	static void binaryCopyToVector_n(std::ifstream & input, std::vector<targetType> & destVec, uint32_t & count)
	{
		sourceType data;
		while (count > 0 && input.read((char *)&data, sizeof(sourceType)))
		{
			destVec.push_back(static_cast<targetType>(data));
			--count;
		}
	}

	//------------------------------------------------------------------------------------------------------
	// get the vertices from file stream and put them into a vector.
	// vertices in vector are continuouse and side-by-side { x1, y1, z1, x2, y2, z2, ... },
	// and if property normal exists { x1, y1, z1, nx1, ny1, nz1, x2, y2, z2, nx2, ny2, nz2, ... },
	// alos true if property euler angle exists { x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2, ... }
	// return empty vector if file format or vertex number is not as expected
	//-------------------------------------------------------------------------------------------------------
	template <typename T>
	static std::vector<T> getVertexList(std::ifstream & input, const Header & header)
	{
		static_assert(std::is_floating_point<T>::value, "only float and double types are supported");

		// make sure input stream is in good state
		input.clear();
		// and not to assume the current position
		if (input.tellg() != header.vertex_position)
			input.seekg(header.vertex_position);

		// the property count must already check to be 3 or 6
		// read data (x, y, z) or (x, y, z, nx, ny, nz) for every vertex
		uint32_t read_count = header.vertex_size * header.property_count;
		std::vector<T> vertices;

		if (header.file_format == Format::binary_little_endian)
		{
			if (header.property_datatype == DataType::Double && !std::is_same<T, double>::value)
				binaryCopyToVector_n<double, T>(input, vertices, read_count);
			else if (header.property_datatype == DataType::Float && !std::is_same<T, float>::value)
				binaryCopyToVector_n<float, T>(input, vertices, read_count);
			else
				binaryCopyToVector_n<T, T>(input, vertices, read_count);
		}
		else if (header.file_format == Format::ascii)
		{
			std::istringstream ssin;
			// redirect file stream buffer to istringstream
			ssin.set_rdbuf(input.rdbuf());
			std::istream_iterator<T> isIterator(ssin);
			std::istream_iterator<T> isEnd;
			/*
			// copy can also get the job done, but we don't want to read garbage if file contains less vertices than it claimed
			std::copy_n(isIterator, read_count, back_inserter(vertices));
			*/
			while (read_count > 0 && isIterator != isEnd)
			{
				vertices.push_back(*isIterator);
				++isIterator;
				--read_count;
			}
			// if all is working out right, read_count should be zero
			if (read_count != 0)
				return std::vector<T>();
		}
		// move vector out of local scope
		return vertices;
	}

public:
	//--------------------------------------------------------------------------------------------------------
	// Read vertex list from PLY file and return a float/double vector
	// vertices in vector are continuouse and side-by-side { x1, y1, z1, x2, y2, z2, ... },
	// and if property normal exists { x1, y1, z1, nx1, ny1, nz1, x2, y2, z2, nx2, ny2, nz2, ... },
	// alos true if property euler angle exists { x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2, ... }
	// return empty vector if file format or vertex number is not as expected
	//--------------------------------------------------------------------------------------------------------
	template<typename T>
	static std::vector<T> readPlyFile(const std::string & filename, CloudMeta & meta)
	{
		static_assert(std::is_floating_point<T>::value, "only float and double types are supported");

		std::ifstream fin(filename, std::ios::in | std::ios::binary);
		if (!fin)
			throw std::runtime_error("failed to open file " + filename + " to read");

		Header header{};
		bool isHeaderOk = parseHeader(fin, header);

		// check if valid header
		if (!isHeaderOk ||
			header.file_format == Format::binary_big_endian ||
			header.vertex_size == 0 ||
			(header.property_count != 3 && header.property_count != 6) ||
			(header.property_datatype != DataType::Float && header.property_datatype != DataType::Double))
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " or vertex list is not stored with expected format";
			throw std::invalid_argument(ssout.str());
		}

		// fill CloudMeta data
		meta.vertex_size = header.vertex_size;
		meta.grid_width = (header.grid_width > 0) ? header.grid_width : header.vertex_size;
		meta.grid_length = (header.grid_length > 0) ? header.grid_length : (header.vertex_size / header.grid_width);
		meta.x_resolution = header.x_resolution;
		meta.y_resolution = header.y_resolution;
		meta.z_resolution = header.z_resolution;
		meta.datetime = header.datetime;
		meta.euler_angle = header.euler_angle;

		// read vertex data and copy to a float/double array
		return getVertexList<T>(fin, header);
	}

	//-------------------------------------------------------------------------------------------------
	// Specialized function to read double type vertex from PLY file into Point3d vector
	// if normal or euler angle properties exists, they are ignored and not included in returned vector
	//-------------------------------------------------------------------------------------------------
	static std::vector<Point3d> readPlyToPoint3d(const std::string & filename, CloudMeta & meta)
	{
		std::vector<double> vertices = readPlyFile<double>(filename, meta);
		if (vertices.empty() || (vertices.size() != (meta.vertex_size * 3) && vertices.size() != (meta.vertex_size * 6)))
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " may not contain valid vertex data";
			throw std::invalid_argument(ssout.str());
		}

		if (vertices.size() == (meta.vertex_size * 3))
		{
			// contain x, y, z points only, convert it to Point3d vector
			// NOTE: it is considered dangerous to assume no hole for the bit layout of Point3d structure
			std::vector<Point3d> pt3dvec(meta.vertex_size);
			std::copy(vertices.begin(), vertices.end(), stdext::make_checked_array_iterator((double *)pt3dvec.data(), vertices.size()));
			// move vector out of local scope
			return pt3dvec;
		}
		else
		{
			// contain x, y, z points with nx, ny, nz normals or rx, ry, rz Euler angles
			// normals and angles are all 3 double structure, skip one Point3d should do it
			std::vector<Point3d> pt3dvec;
			Point3d* ppt3d = (Point3d*)vertices.data();
			for (uint32_t i = 0, end = meta.vertex_size * 2; i < end; i += 2)
				pt3dvec.push_back(*(ppt3d + i));
			// move vector out of local scope
			return pt3dvec;
		}
	}

	//-------------------------------------------------------------------------------------------------
	// Specialized function to read float type vertex from PLY file into Point3f vector
	// if normal or euler angle properties exists, they are ignored and not included in returned vector
	//-------------------------------------------------------------------------------------------------
	static std::vector<Point3f> readPlyToPoint3f(const std::string & filename, CloudMeta & meta)
	{
		std::vector<float> vertices = readPlyFile<float>(filename, meta);
		if (vertices.empty() || (vertices.size() != (meta.vertex_size * 3) && vertices.size() != (meta.vertex_size * 6)))
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " may not contain valid vertex data";
			throw std::invalid_argument(ssout.str());
		}
		// convert to Point3f vector
		if (vertices.size() == (meta.vertex_size * 3))
		{
			// contain x, y, z points only, convert it to Point3f vector
			// NOTE: it is considered dangerous to assume no hole for the bit layout of Point3f structure
			std::vector<Point3f> pt3fvec(meta.vertex_size);
			std::copy(vertices.begin(), vertices.end(), stdext::make_checked_array_iterator((float *)pt3fvec.data(), vertices.size()));
			return pt3fvec;
		}
		else
		{
			// contain x, y, z points with nx, ny, nz normals or rx, ry, rz Euler angles
			// normals and angles are all 3 float structure, skip one Point3f should do it
			std::vector<Point3f> pt3fvec;
			Point3f* ppt3f = (Point3f*)vertices.data();
			for (uint32_t i = 0, end = meta.vertex_size * 2; i < end; i += 2)
				pt3fvec.push_back(*(ppt3f + i));
			return pt3fvec;
		}
		// move vector out of local scope
	}

	//-------------------------------------------------------------------------------------------
	// Specialized function to read double type vertex from PLY file into Vertex6d vector
	// if normal or euler angle properties are not exists, they are left with zero values
	//-------------------------------------------------------------------------------------------
	static std::vector<Vertex6d> readPlyToVertex6d(const std::string & filename, CloudMeta & meta)
	{
		std::vector<double> vertices = readPlyFile<double>(filename, meta);
		if (vertices.empty() || (vertices.size() != (meta.vertex_size * 3) && vertices.size() != (meta.vertex_size * 6)))
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " may not contain valid vertex data";
			throw std::invalid_argument(ssout.str());
		}

		std::vector<Vertex6d> vtx6dvec(meta.vertex_size);
		if (vertices.size() == (meta.vertex_size * 6))
		{
			// contain x, y, z points with nx, ny, nz normals or rx, ry, rz Euler angles
			// NOTE: it is considered dangerous to assume no hole for the bit layout of Point3d structure
			std::copy(vertices.begin(), vertices.end(), stdext::make_checked_array_iterator((double *)vtx6dvec.data(), vertices.size()));
		}
		else
		{
			// contain x, y, z points only, copy only to Point3d
			Point3d* ppt3d = (Point3d*)vertices.data();
			for (uint32_t i = 0; i < meta.vertex_size; ++i)
				vtx6dvec[i] = { *(ppt3d + i) , { .0, .0, .0 } };
		}
		// move vector out of local scope
		return vtx6dvec;
	}

	//-------------------------------------------------------------------------------------------
	// Specialized function to read double type vertex from PLY file into Vertex6f vector
	// if normal or euler angle properties are not exists, they are left with zero values
	//-------------------------------------------------------------------------------------------
	static std::vector<Vertex6f> readPlyToVertex6f(const std::string & filename, CloudMeta & meta)
	{
		std::vector<float> vertices = readPlyFile<float>(filename, meta);
		if (vertices.empty() || (vertices.size() != (meta.vertex_size * 3) && vertices.size() != (meta.vertex_size * 6)))
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " may not contain valid vertex data";
			throw std::invalid_argument(ssout.str());
		}

		std::vector<Vertex6f> vtx6fvec(meta.vertex_size);
		if (vertices.size() == (meta.vertex_size * 6))
		{
			// contain x, y, z points with nx, ny, nz normals or rx, ry, rz Euler angles
			// NOTE: it is considered dangerous to assume no hole for the bit layout of Point3d structure
			std::copy(vertices.begin(), vertices.end(), stdext::make_checked_array_iterator((float *)vtx6fvec.data(), vertices.size()));
		}
		else
		{
			// contain x, y, z points only, copy only to Point3d
			Point3f* ppt3f = (Point3f*)vertices.data();
			for (uint32_t i = 0; i < meta.vertex_size; ++i)
				vtx6fvec[i] = { *(ppt3f + i) , { .0, .0, .0 } };
		}
		// move vector out of local scope
		return vtx6fvec;
	}

	//-------------------------------------------------------------------------------------------------
	// Specialized function to read double type vertex from PLY file into Eigen::MatrixX3d
	// if normal or euler angle properties exists, they are ignored and not included in returned mztrix
	//-------------------------------------------------------------------------------------------------
	static Eigen::MatrixX3d readPlyToMatrixX3d(const std::string & filename, CloudMeta & meta)
	{
		std::vector<double> vertices = readPlyFile<double>(filename, meta);
		if (vertices.empty() || (vertices.size() != (meta.vertex_size * 3) && vertices.size() != (meta.vertex_size * 6)))
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " may not contain valid vertex data";
			throw std::invalid_argument(ssout.str());
		}

		if (vertices.size() == (meta.vertex_size * 3))
		{
			// contain x, y, z points only, mapping to Eigen::MatrixX3d
			Eigen::Map<Eigen::Matrix<double, -1, 3, Eigen::RowMajor>> mat3d(vertices.data(), vertices.size() / 3, 3);
			// move matrix out of local scope
			return mat3d;
		}
		else
		{
			// contain x, y, z points with nx, ny, nz normals or rx, ry, rz Euler angles
			// normals and angles are all 3 double structure, skip one Point3d should do it
			Eigen::MatrixX3d mat3d(meta.vertex_size, 3);
			Point3d* ppt3d = (Point3d*)vertices.data();
			for (uint32_t r = 0; r < meta.vertex_size; ++r, ppt3d += 2)
				mat3d.row(r) << ppt3d->x, ppt3d->y, ppt3d->z;
			// move vector out of local scope
			return mat3d;
		}
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to read float type vertex from PLY file into Eigen::MatrixX3f
	//-------------------------------------------------------------------------------------
	static Eigen::MatrixX3f readPlyToMatrixX3f(const std::string & filename, CloudMeta & meta)
	{
		std::vector<float> vertices = readPlyFile<float>(filename, meta);
		if (vertices.empty() || (vertices.size() != (meta.vertex_size * 3) && vertices.size() != (meta.vertex_size * 6)))
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " may not contain valid vertex data";
			throw std::invalid_argument(ssout.str());
		}

		if (vertices.size() == (meta.vertex_size * 3))
		{
			// mapping to Eigen::MatrixX3f
			Eigen::Map<Eigen::Matrix<float, -1, 3, Eigen::RowMajor>> mat3f(vertices.data(), vertices.size() / 3, 3);
			// move matrix out of local scope
			return mat3f;
		}
		else
		{
			// contain x, y, z points with nx, ny, nz normals or rx, ry, rz Euler angles
			// normals and angles are all 3 double structure, skip one Point3d should do it
			Eigen::MatrixX3f mat3f(meta.vertex_size, 3);
			Point3f* ppt3f = (Point3f*)vertices.data();
			for (uint32_t r = 0; r < meta.vertex_size; ++r, ppt3f += 2)
				mat3f.row(r) << ppt3f->x, ppt3f->y, ppt3f->z;
			// move matrix out of local scope
			return mat3f;
		}
	}

	//-------------------------------------------------------------------------------------------------------
	// Write vertex list from a float/double vector to a PLY file
	// vertices in vector are continuouse and side-by-side { x1, y1, z1, x2, y2, z2, ... } 
	// and if property normal exists { x1, y1, z1, nx1, ny1, nz1, x2, y2, z2, nx2, ny2, nz2, ... },
	// alos true if property euler angle exists { x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2, ... }
	//-------------------------------------------------------------------------------------------------------
	template<typename T>
	static void writePlyFile(const std::string & filename, CloudMeta & meta, std::vector<T> vertices, bool binary_format = true)
	{
		static_assert(std::is_floating_point<T>::value, "only float and double types are supported");

		std::ofstream fout(filename, std::ios::out | std::ios::binary);
		if (!fout)
			throw std::runtime_error("failed to open file " + filename + " to write");

		// don't bother to write anything if nothing need to write
		if (meta.vertex_size == 0 || vertices.size() == 0)
			return;

		bool hasNormalOrAngle = (vertices.size() == (meta.vertex_size * 6)) ? true : false;
		// construct PLY header
		std::ostringstream header;
		header << "ply\n"
			<< "format " << ((binary_format) ? "binary_little_endian" : "ascii") << " 1.0\n"
			<< "comment Point Cloud (Orisol Asia Ltd.)\n"
			<< "element vertex " << meta.vertex_size << "\n";
		if (!meta.datetime.empty())
			header << "comment datetime " << meta.datetime << "\n";
		if (!meta.euler_angle.empty())
			header << "comment euler " << meta.euler_angle << "\n";
		if (meta.x_resolution > 0 && meta.y_resolution > 0 && meta.z_resolution > 0)
			header << "comment resolution " << meta.x_resolution << " " << meta.y_resolution << " " << meta.z_resolution << "\n";
		header << "comment dimension " << meta.grid_width << " " << meta.grid_length << "\n"
			<< "property " << typeid(T).name() << " x\n"
			<< "property " << typeid(T).name() << " y\n"
			<< "property " << typeid(T).name() << " z\n";
		if (hasNormalOrAngle)
		{
			if (meta.euler_angle.empty())
				header << "property " << typeid(T).name() << " nx\n"
					<< "property " << typeid(T).name() << " ny\n"
					<< "property " << typeid(T).name() << " nz\n";
			else
				header << "property " << typeid(T).name() << " rx\n"
					<< "property " << typeid(T).name() << " ry\n"
					<< "property " << typeid(T).name() << " rz\n";
		}
		header << "end_header\n";
		// write header to file
		fout << header.str();

		// write vertices to file
		if (binary_format)
		{
			// output vertices as binary data
			// Note: NOT portable here, little endian is assumed  
			fout.write((char *)vertices.data(), vertices.size() * sizeof(T));
		}
		else
		{
			// output vertices as ASCII text
			uint32_t counter = 0;
			uint32_t newline_every_n = (hasNormalOrAngle) ? 6 : 3;
			for (const auto& coordinate : vertices)
			{
				fout << coordinate;
				// one newline for every x, y, z group
				fout << ((++counter % newline_every_n) == 0 ? "\n" : " ");
			}
		}
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to write Point3d vector to PLY file
	// all elements of vector are points, no normal or angle is allowed
	//-------------------------------------------------------------------------------------
	static void writePoint3dToPly(const std::string & filename, CloudMeta & meta, std::vector<Point3d> & pt3vec, bool binary_format = true)
	{
		if (pt3vec.size() != meta.vertex_size)
			throw std::invalid_argument("the size of vector is not the same with meta");
		meta.euler_angle.clear();
		std::vector<double> vertices((double *)pt3vec.data(), (double *)pt3vec.data() + (pt3vec.size() * 3));
		writePlyFile<double>(filename, meta, std::move(vertices), binary_format);
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to write Point3f vector to PLY file
	// all elements of vector are points, no normal or angle is allowed
	//-------------------------------------------------------------------------------------
	static void writePoint3fToPly(const std::string & filename, CloudMeta & meta, std::vector<Point3f> & pt3vec, bool binary_format = true)
	{
		if (pt3vec.size() != meta.vertex_size)
			throw std::invalid_argument("the size of vector is not the same with meta");
		meta.euler_angle.clear();
		std::vector<float> vertices((float *)pt3vec.data(), (float *)pt3vec.data() + (pt3vec.size() * 3));
		writePlyFile<float>(filename, meta, std::move(vertices), binary_format);
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to write Vertex6d vector to PLY file
	//-------------------------------------------------------------------------------------
	static void writeVertex6dToPly(const std::string & filename, CloudMeta & meta, std::vector<Vertex6d> & vtx3vec, bool binary_format = true)
	{
		std::vector<double> vertices((double *)vtx3vec.data(), (double *)vtx3vec.data() + (vtx3vec.size() * 6));
		writePlyFile<double>(filename, meta, std::move(vertices), binary_format);
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to write Vertex6f vector to PLY file
	//-------------------------------------------------------------------------------------
	static void writeVertex6fToPly(const std::string & filename, CloudMeta & meta, std::vector<Vertex6f> & vtx3vec, bool binary_format = true)
	{
		std::vector<float> vertices((float *)vtx3vec.data(), (float *)vtx3vec.data() + (vtx3vec.size() * 6));
		writePlyFile<float>(filename, meta, std::move(vertices), binary_format);
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to write Eigen::MatrixX3d to PLY file
	//-------------------------------------------------------------------------------------
	static void writeMatrixX3dToPly(const std::string & filename, CloudMeta & meta, Eigen::MatrixX3d & mat3d, bool binary_format = true)
	{
		if (mat3d.rows() != meta.vertex_size)
			throw std::invalid_argument("the number of row is not the same with meta");

		std::vector<double> vertices(mat3d.size());
		if (mat3d.IsRowMajor)
		{
			std::copy(mat3d.data(), mat3d.data() + mat3d.size(), vertices.begin());
		}
		else
		{
			Eigen::Matrix<double, -1, 3, Eigen::RowMajor> mat3d_rowmajor = mat3d;
			std::copy(mat3d_rowmajor.data(), mat3d_rowmajor.data() + mat3d_rowmajor.size(), vertices.begin());
		}
		writePlyFile<double>(filename, meta, std::move(vertices), binary_format);
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to write Eigen::MatrixX3f to PLY file
	//-------------------------------------------------------------------------------------
	static void writeMatrixX3fToPly(const std::string & filename, CloudMeta & meta, Eigen::MatrixX3f & mat3d, bool binary_format = true)
	{
		if (mat3d.rows() != meta.vertex_size)
			throw std::invalid_argument("the number of row is not the same with meta");

		std::vector<float> vertices(mat3d.size());
		if (mat3d.IsRowMajor)
		{
			std::copy(mat3d.data(), mat3d.data() + mat3d.size(), vertices.begin());
		}
		else
		{
			Eigen::Matrix<float, -1, 3, Eigen::RowMajor> mat3d_rowmajor = mat3d;
			std::copy(mat3d_rowmajor.data(), mat3d_rowmajor.data() + mat3d_rowmajor.size(), vertices.begin());
		}
		writePlyFile<float>(filename, meta, std::move(vertices), binary_format);
	}
};

class XyzFile
{
private:
	struct Header
	{
		uint32_t vertex_size;
		uint32_t grid_width;
		uint32_t grid_length;
		std::streampos vertex_position;
	};

	//------------------------------------------------------------------------------------
	// parse the header of XYZ file from input stream
	// return false if not an expected XYZ file header format
	//------------------------------------------------------------------------------------
	static bool parseHeader(std::ifstream & input, Header & header)
	{
		std::string aline;
		// get the first line
		if (!std::getline(input, aline))
			return false;

		std::string token1, token2, token3;
		std::istringstream(aline) >> token1 >> token2 >> token3;
		if (!token1.empty() && token2.empty() && token3.empty())
			// the first single number line is the number of profiles
			header.grid_length = std::stoul(token1);
		else
			return false;

		// get the second line
		if (!std::getline(input, aline))
			return false;

		std::istringstream(aline) >> token1 >> token2 >> token3;
		if (!token1.empty() && token2.empty() && token3.empty())
			// the second single number line is the number of points for each profile
			header.grid_width = std::stoul(token1);
		else
			return false;

		// fill in the rest of header
		header.vertex_size = header.grid_length * header.grid_width;
		header.vertex_position = input.tellg();
	}

public:
	//-------------------------------------------------------------------------------------
	// Read vertex list from XYZ file and return a float/double vector
	// vertices in vector are continuouse and side-by-side { x1, y1, z1, x2, y2, z2, ... } 
	//-------------------------------------------------------------------------------------
	template<typename T>
	static std::vector<T> readXyzFile(const std::string & filename, CloudMeta & meta)
	{
		static_assert(std::is_floating_point<T>::value, "only float and double types are supported");

		std::ifstream fin(filename, std::ios::in | std::ios::binary);
		if (!fin)
			throw std::runtime_error("failed to open file " + filename + " to read");

		Header header{};
		bool isHeaderOk = parseHeader(fin, header);

		// check if valid header exist
		if (!isHeaderOk)
		{
			fin.seekg(0, std::ios::beg);
			header.vertex_position = fin.tellg();
		}

		fin.clear();
		std::vector<T> vertices;
		std::string aline;
		while (std::getline(fin, aline))
		{
			// expect 3 numeric string in a line
			std::string ax1, ax2, ax3;
			std::istringstream(aline) >> ax1 >> ax2 >> ax3;
			if (!ax1.empty() && !ax2.empty() && !ax3.empty())
			{
				vertices.push_back(static_cast<T>(std::stod(ax1)));
				vertices.push_back(static_cast<T>(std::stod(ax2)));
				vertices.push_back(static_cast<T>(std::stod(ax3)));
			}
		}

		if (!isHeaderOk)
		{
			meta.vertex_size = vertices.size() / 3;
			meta.grid_width = meta.vertex_size;
			meta.grid_length = (meta.vertex_size > meta.grid_width) ? meta.vertex_size / meta.grid_width : 1;
		}
		else
		{
			meta.vertex_size = header.vertex_size;
			meta.grid_width = header.grid_width;
			meta.grid_length = header.grid_length;
		}
		meta.x_resolution = .0;
		meta.y_resolution = .0;
		meta.z_resolution = .0;

		return vertices;
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to read double type vertex from XYZ file into Point3d vector
	//-------------------------------------------------------------------------------------
	static std::vector<Point3d> readXyzToPoint3d(const std::string & filename, CloudMeta & meta)
	{
		std::vector<double> vertices = readXyzFile<double>(filename, meta);
		if (vertices.empty())
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " may not contain valid vertex data";
			throw std::invalid_argument(ssout.str());
		}
		// convert to Point3d vector
		std::vector<Point3d> pt3dvec(vertices.size() / 3);
		// NOTE: it is considered dangerous to assume no hole for the bit layout of Point3d structure
		std::copy(vertices.begin(), vertices.end(), stdext::make_checked_array_iterator((double *)pt3dvec.data(), vertices.size()));
		// move vector out of local scope
		return pt3dvec;
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to read float type vertex from Xyz file into Point3f vector
	//-------------------------------------------------------------------------------------
	static std::vector<Point3f> readXyzToPoint3f(const std::string & filename, CloudMeta & meta)
	{
		std::vector<float> vertices = readXyzFile<float>(filename, meta);
		if (vertices.empty())
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " may not contain valid vertex data";
			throw std::invalid_argument(ssout.str());
		}
		// convert to Point3f vector
		std::vector<Point3f> pt3fvec(vertices.size() / 3);
		// NOTE: it is considered dangerous to assume no hole for the bit layout of Point3f structure
		std::copy(vertices.begin(), vertices.end(), stdext::make_checked_array_iterator((float *)pt3fvec.data(), vertices.size()));
		// move vector out of local scope
		return pt3fvec;
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to read double type vertex from XYZ file into Eigen::MatrixX3d
	//-------------------------------------------------------------------------------------
	static Eigen::MatrixX3d readXyzToMatrixX3d(const std::string & filename, CloudMeta & meta)
	{
		std::vector<double> vertices = readXyzFile<double>(filename, meta);
		if (vertices.empty())
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " may not contain valid vertex data";
			throw std::invalid_argument(ssout.str());
		}
		// mapping to Eigen::MatrixX3d
		Eigen::Map<Eigen::Matrix<double, -1, 3, Eigen::RowMajor>> mat3d(vertices.data(), vertices.size() / 3, 3);
		// move matrix out of local scope
		return mat3d;
	}

	//-------------------------------------------------------------------------------------
	// Specialized function to read float type vertex from Xyz file into Eigen::MatrixX3f
	//-------------------------------------------------------------------------------------
	static Eigen::MatrixX3f readXyzToMatrixX3f(const std::string & filename, CloudMeta & meta)
	{
		std::vector<float> vertices = readXyzFile<float>(filename, meta);
		if (vertices.empty())
		{
			std::ostringstream ssout;
			ssout << "input file " << filename << " may not contain valid vertex data";
			throw std::invalid_argument(ssout.str());
		}
		// mapping to Eigen::MatrixX3f
		Eigen::Map<Eigen::Matrix<float, -1, 3, Eigen::RowMajor>> mat3f(vertices.data(), vertices.size() / 3, 3);
		// move matrix out of local scope
		return mat3f;
	}
};