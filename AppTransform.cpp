#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <type_traits>
#include <iterator>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <locale>
#include <codecvt>
#include <Poco/Util/Option.h>
#include <Poco/Util/HelpFormatter.h>
#include <Poco/Util/AbstractConfiguration.h>
#include <Poco/File.h>
#include <Poco/Format.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include "AppTransform.h"
#include "Point3dUtils.hpp"

using std::string;
using std::ifstream;
using std::ostringstream;
using std::vector;
using std::istream_iterator;
using std::back_inserter;
using std::to_string;
using Poco::Util::Application;
using Poco::Util::Option;
using Poco::Util::OptionSet;
using Poco::Util::OptionCallback;
using Poco::Util::HelpFormatter;
using Poco::Util::AbstractConfiguration;
using Poco::File;
using Poco::format;

void AppTransform::handleOptionHelp(const string & option, const string & argument)
{
	poco_trace(logger(), "handleOptionHelp: " + option + "=" + argument);
	_helpRequested = true;
	// display help
	HelpFormatter helpFormatter(options());
	helpFormatter.setCommand(commandName());
	helpFormatter.setUsage("OPTIONS");
	helpFormatter.setHeader("Spatial transform a set of points, angle, or transform matrix estimation from 2 sets of point");
	helpFormatter.format(std::cout);
	// stop further processing
	stopOptionsProcessing();
}

void AppTransform::initialize(Application & self)
{
	poco_trace(logger(), config().getString("application.baseName", name()) + " initialize");
	// load default configuration file
	loadConfiguration();
	// all registered subsystems are initialized in ancestor's initialize procedure
	Application::initialize(self);
}

void AppTransform::uninitialize()
{
	poco_trace(logger(), config().getString("application.baseName", name()) + " uninitialize");
	// ancestor uninitialization
	Application::uninitialize();
}

void AppTransform::defineOptions(Poco::Util::OptionSet & options)
{
	Application::defineOptions(options);

	options.addOption(
		Option("help", "h", "(/h) display help information on command line arguments")
		.required(false)
		.repeatable(false)
		.callback(OptionCallback<AppTransform>(this, &AppTransform::handleOptionHelp)));

	ostringstream optstrs;
	optstrs << "(/@) specify the action to take for transformation, available action types are:\n"
		<< "'display' - display SOURCE or TRANSFORM matrix information\n"
		<< "'estimate' - estimate TRANSFORM from SOURCE to TARGET\n"
		<< "'build' - build transform matrix from LINEAR, ROTATION, TRANSLATION, and SCALING\n"
		<< "'transform' - apply TRANSFORM matrix on SOURCE and ANGLE matrix (.matrix or .ply)\n";
	// This option assign argument directly to configuration "application.action".
	options.addOption(
		Option("@ction", "@", optstrs.str())
		.required(true)
		.repeatable(false)
		.argument("type")
		.binding("application.action"));

	// This option assign argument directly to configuration "application.source.matrix".
	options.addOption(
		Option("matrix", "m", "(/m) SOURCE data from .matrix file")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.source.matrix"));

	// This option assign argument directly to configuration "application.source.ply".
	options.addOption(
		Option("ply", "p", "(/p) SOURCE data from .ply file")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.source.ply"));

	// This option assign argument directly to configuration "application.target.matrix".
	options.addOption(
		Option("destMatrix", "d", "(/d) TARGET data from .matrix file")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.target.matrix"));

	// This option assign argument directly to configuration "application.transform.matrix"
	options.addOption(
		Option("xformMatrix", "x", "(/x) TRANSFORM matrix from .matrix file")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.transform.matrix"));

	// This option assign argument directly to configuration "application.linear.matrix".
	options.addOption(
		Option("linear", "l", "(/l) LINEAR matrix from .matrix file")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.linear.matrix"));

	// This option assign argument directly to configuration "application.angle.matrix".
	options.addOption(
		Option("rotation", "r", "(/r) rotation ANGLE (Rx, Ry, Rz) vector from .matrix file to construct ROTATION matrix")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.angle.matrix"));

	// This option assign argument directly to configuration "application.translation.matrix".
	options.addOption(
		Option("translation", "t", "(/t) TRANSLATION (Tx, Ty, Tz) vector from .matrix file")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.translation.matrix"));

	// This option assign argument directly to configuration "application.scaling.matrix".
	options.addOption(
		Option("scaling", "s", "(/s) SCALING (Sx, Sy, Sz) vector from .matrix file, or boolean selection for transform matrix estimation")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.scaling.matrix"));

	// This option assign argument directly to configuration "application.build.order".
	optstrs.str("");
	optstrs.seekp(0);
	optstrs << "(/o) the order to build or apply transformation matrix.\n"
		<< "*Build* order example: 'rIst', where I is the identity matrix. The option LINEAR matrix, if present, is always applied first to the identity."
		<< "*Apply* order example: 'xI' to apply xform on the left, and 'Ix' to apply xform on the right.";
	options.addOption(
		Option("order", "o", optstrs.str())
		.required(false)
		.repeatable(false)
		.argument("sequence")
		.binding("application.transform.order"));
}

//============================================================================================
// .matrix is a simple ascii text format file for matrix data
// the first line of .matrix file must contain 2 numbers: rows and columns 
// each line contain elements of a row, and assumed to have 3 columns for geometric position.
// for position data, columns are x, y, z respectively in a row.
// for rotation angle, columns are rx, ry, rz respectively in a row.
//============================================================================================
template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> loadMatrix(string filename)
{
	static_assert(std::is_floating_point<T>::value, "only floating point types are supported");

	using MatrixType = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

	// convert filename from UTF-8 string to UTF-16 wstring
	std::wstring_convert<std::codecvt_utf8<wchar_t>> convert2wstring;
	std::wstring wfilename = convert2wstring.from_bytes(filename);
	ifstream fin(wfilename, std::ios::in | std::ios::binary);
	if (!fin)
		throw std::runtime_error("failed to open file " + filename + " to read");

	// read the dimension line
	string aline;
	uint32_t rows, cols;
	if (!std::getline(fin, aline))
		throw std::runtime_error("failed to open file " + filename + " to read");
	std::istringstream(aline) >> rows >> cols;

	vector<T> elements;
	std::copy(istream_iterator<T>(fin), istream_iterator<T>(), back_inserter(elements));
	// the number of elements must match rows * cols
	if (elements.size() != (rows * cols))
		throw std::runtime_error("the number of element is incorrect in " + filename);

	// map to matrix
	Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> matrix((T*)elements.data(), rows, cols);
	return matrix;
}

int AppTransform::main(const ArgVec & args)
{
	if (_helpRequested)
		return Application::EXIT_USAGE;

	string action = config().getString("application.action", "");
	string sourceMatrix = config().getString("application.source.matrix", "");
	string sourcePly = config().getString("application.source.ply", "");
	string targetMatrix = config().getString("application.target.matrix", "");
	string xformMatrix = config().getString("application.transform.matrix", "");
	string linearMatrix = config().getString("application.linear.matrix", "");
	string angleMatrix = config().getString("application.angle.matrix", "");
	string translateMatrix = config().getString("application.translation.matrix", "");
	string scalingMatrix = config().getString("application.scaling.matrix", "");
	string xformOrder = config().getString("application.transform.order", "I");

	//----------------------------------------------------------------------------
	// display data or certain transformation information
	// Note: targetMatrix is not used on display action
	//----------------------------------------------------------------------------
	if (action == "display")
	{
		//----------------------------------------------------------------------------
		// display available information about TRANSFORM matrix
		//----------------------------------------------------------------------------
		if (!xformMatrix.empty() && File(xformMatrix).exists())
		{
			// transformation matrix is 4 x 4 matrix
			Eigen::Affine3d xform;
			try
			{
				Eigen::MatrixXd matTrans = loadMatrix<double>(xformMatrix);
				xform.matrix() = matTrans;
			}
			catch (std::exception & e)
			{
				poco_error(logger(), string(e.what()));
				return Application::EXIT_DATAERR;
			}
			ostringstream ostrs;
			ostrs << "### TRANSFORM matrix:\n" << xform.matrix() << "\n"
				<< "--- >>> Affine part:\n" << xform.affine() << "\n"
				<< "--- >>> Rotation part:\n" << xform.rotation() << "\n"
				<< "--- >>> Translation part:\n" << xform.translation() << "\n"
				<< "--- >>> Inverse:\n" << xform.inverse().matrix() << "\n";
			std::cout << ostrs.str() << std::endl;
		}
		//----------------------------------------------------------------------------
		// display content of SOURCE matrix from .matrix file
		//----------------------------------------------------------------------------
		else if (!sourceMatrix.empty() && File(sourceMatrix).exists())
		{
			Eigen::MatrixXd matSource;
			try
			{
				matSource = loadMatrix<double>(sourceMatrix);
			}
			catch (std::exception & e)
			{
				poco_error(logger(), string(e.what()));
				return Application::EXIT_DATAERR;
			}
			std::cout << "### SOURCE matrix:\n" << matSource << std::endl;
		}
		//----------------------------------------------------------------------------
		// display content of SOURCE matrix from .ply file
		//----------------------------------------------------------------------------
		else if (!sourcePly.empty() && File(sourcePly).exists())
		{
			CloudMeta meta;
			vector<Vertex6d> v6dSource;
			try
			{
				v6dSource = PlyFile::readPlyToVertex6d(sourcePly, meta);
			}
			catch (std::exception & e)
			{
				poco_error(logger(), string(e.what()));
				return Application::EXIT_DATAERR;
			}
			ostringstream ostrs;
			ostrs << "### SOURCE matrix from ply:\n"
				<< "--- >>> size = " << meta.vertex_size << "\n"
				<< "--- >>> width = " << meta.grid_width << "\n"
				<< "--- >>> length = " << meta.grid_length << "\n";
			if (!meta.euler_angle.empty())
				ostrs << "--- >>> euler = " << meta.euler_angle << "\n";
			if (!meta.datetime.empty())
				ostrs << "--- >>> datetime = " << meta.datetime << "\n";
			std::cout << ostrs.str() << std::endl;
			for (const auto & vertex : v6dSource)
				std::cout
				<< std::setw(12) << vertex.point.x
				<< std::setw(12) << vertex.point.y
				<< std::setw(12) << vertex.point.z
				<< std::setw(12) << vertex.angle.rx
				<< std::setw(12) << vertex.angle.ry
				<< std::setw(12) << vertex.angle.rz;
		}
	}
	//----------------------------------------------------------------------------
	// Transform matrix estimation using umeyama
	// Note: both SOURCE and TARGET are read from .matrix file
	//----------------------------------------------------------------------------
	else if (action == "estimate")
	{
		if (sourceMatrix.empty() || targetMatrix.empty())
			throw std::invalid_argument("SOURCE and TARGET matrix must not be empty");

		Eigen::MatrixXd matTarget = loadMatrix<double>(targetMatrix);
		matTarget.transposeInPlace();

		Eigen::MatrixXd matSource = loadMatrix<double>(sourceMatrix);
		matSource.transposeInPlace();

		std::cout << "Source Matrix:\n" << matSource << std::endl;
		std::cout << "Destination Matrix:\n" << matTarget << std::endl;

		if (matSource.rows() != matTarget.rows() || matSource.cols() != matTarget.cols())
			throw std::invalid_argument("the dimension of SOURCE and TARGET matrix must match");

		if (matSource.rows() < 2 || matSource.cols() < 2)
			throw std::invalid_argument("the dimension of matrix must at least 2x2");

		bool scaling = !scalingMatrix.empty();
		Eigen::MatrixXd cR_t = Eigen::umeyama(matSource, matTarget, scaling);
		double error = (cR_t * matSource.colwise().homogeneous() - matTarget.colwise().homogeneous()).squaredNorm();
		std::cout << "Transform Matrix:\n" << cR_t << std::endl;
		//std::cout << "Transform Result:\n" << cR_t * sourceMatrix.colwise().homogeneous() << std::endl;
		std::cout << "estimation error = " << error << std::endl;
		std::cout << "where epsilon is " << std::numeric_limits<double>::epsilon() << "\n" << std::endl;
	}
	//-------------------------------------------------------------------------------------------------
	// display the transform matrix built from combination of ROTATION, ANGLE, TRANSLATION, and SCALING
	//-------------------------------------------------------------------------------------------------
	else if (action == "build")
	{
		// transformation matrix is 4 x 4 matrix
		Eigen::Affine3d xform(Eigen::Affine3d::Identity());
		ostringstream ostrs;
		ostrs << "### build TRANSFORM matrix:\n";
		try
		{
			if (!linearMatrix.empty())
			{
				Eigen::MatrixXd matLinear = loadMatrix<double>(linearMatrix);
				if (matLinear.rows() != 3 || matLinear.cols() != 3)
					throw std::runtime_error("ROTATION matrix (" + to_string(matLinear.rows()) + "x" + to_string(matLinear.cols()) + ") has wrong dimension");
				// directly put into linear part
				xform.linear() = matLinear;
				ostrs << "--- >>> from LINEAR matrix:\n" << matLinear << "\n";
			}

			// rotation matrix
			Eigen::Matrix3d xRotate;
			if (!angleMatrix.empty())
			{
				Eigen::MatrixXd matAngle = loadMatrix<double>(angleMatrix);
				if (matAngle.size() < 3 || matAngle.cols() != 3)
					throw std::runtime_error("ANGLE matrix (" + to_string(matAngle.rows()) + "x" + to_string(matAngle.cols()) + ") has no enough elements");
				// take only the 1st row to build
				xRotate = Eigen::AngleAxisd(matAngle.row(0).z() * EIGEN_PI / 180, Eigen::Vector3d::UnitZ())
					* Eigen::AngleAxisd(matAngle.row(0).y() * EIGEN_PI / 180, Eigen::Vector3d::UnitY())
					* Eigen::AngleAxisd(matAngle.row(0).x() * EIGEN_PI / 180, Eigen::Vector3d::UnitX());
			}
			else
			{
				xRotate = Eigen::Matrix3d::Identity();
			}

			// scaling matrix
			//Eigen::DiagonalMatrix<double, 3> xScale;
			Eigen::Vector3d xScale;
			if (!scalingMatrix.empty())
			{
				Eigen::MatrixXd matScaling = loadMatrix<double>(scalingMatrix);
				if (matScaling.size() < 3 || matScaling.cols() != 3)
					throw std::runtime_error("SCALING matrix (" + to_string(matScaling.rows()) + "x" + to_string(matScaling.cols()) + ") has no enough elements");
				// take only the 1st row to build
				xScale = matScaling.row(0).transpose();
			}
			else
			{
				xScale = Eigen::Vector3d(1.0, 1.0, 1.0);
			}

			// translation matrix
			//Eigen::Translation3d xTranslate;
			Eigen::Vector3d xTranslate;
			if (!translateMatrix.empty())
			{
				Eigen::MatrixXd matTranslate = loadMatrix<double>(translateMatrix);
				if (matTranslate.size() < 3 || matTranslate.cols() != 3)
					throw std::runtime_error("TRANSLATION matrix (" + to_string(matTranslate.rows()) + "x" + to_string(matTranslate.cols()) + ") has no enough elements");
				// take only the 1st row to build
				xTranslate = matTranslate.row(0).transpose();
			}
			else
			{
				xTranslate = Eigen::Vector3d::Zero();
			}

			// convert build order string to all upper case
			std::transform(xformOrder.begin(), xformOrder.end(), xformOrder.begin(), [](char c) { return std::toupper(c, std::locale::classic()); });
			string::size_type idxI = xformOrder.find("I");
			if (idxI != string::npos)
			{
				int idxCode = (int)idxI;
				// to the left
				while (--idxCode >= 0)
				{
					if (xformOrder.at(idxCode) == 'R')
					{
						//xform = xRotate * xform;
						xform.prerotate(xRotate);
						ostrs << "--- >>> Pre-ROTATE:\n" << xRotate << "\n";
					}
					else if (xformOrder.at(idxCode) == 'S')
					{
						//xform = xScale * xform;
						 xform.prescale(xScale);
						ostrs << "--- >>> Pre-SCALE:\n" << xScale << "\n";
					}
					else if (xformOrder.at(idxCode) == 'T')
					{
						//xform = xTranslate * xform;
						xform.pretranslate(xTranslate);
						ostrs << "--- >>> Pre-TRANSLATE:\n" << xTranslate << "\n";
					}
				}
				// to the right
				idxCode = (int)idxI;
				while (++idxCode < xformOrder.length())
				{
					if (xformOrder.at(idxCode) == 'R')
					{
						//xform *= xRotate;
						xform.rotate(xRotate);
						ostrs << "--- >>> ROTATE:\n" << xRotate << "\n";
					}
					else if (xformOrder.at(idxCode) == 'S')
					{
						//xform *= xScale;
						xform.scale(xScale);
						ostrs << "--- >>> SCALE:\n" << xScale << "\n";
					}
					else if (xformOrder.at(idxCode) == 'T')
					{
						//xform *= xTranslate;
						xform.translate(xTranslate);
						ostrs << "--- >>> TRANSLATE:\n" << xTranslate << "\n";
					}
				}
			}
			else
			{
				// xform.prerotate(xRotate).scale(xScale).translate(xTranslate);
				xform.prerotate(xRotate);
				ostrs << "--- >>> Pre-ROTATE:\n" << xRotate << "\n";
				xform.scale(xScale);
				ostrs << "--- >>> SCALE:\n" << xScale << "\n";
				xform.translate(xTranslate);
				ostrs << "--- >>> TRANSLATE:\n" << xTranslate << "\n";
			}
		}
		catch (std::exception & e)
		{
			poco_error(logger(), string(e.what()));
			return Application::EXIT_DATAERR;
		}
		ostrs << "=== >>> result TRANSFORM matrix:\n" << xform.matrix() << "\n"
			<< "=== >>> Affine part:\n" << xform.affine() << "\n"
			<< "=== >>> Rotation part:\n" << xform.rotation() << "\n"
			<< "=== >>> Translation part:\n" << xform.translation() << "\n"
			<< "=== >>> Inverse:\n" << xform.inverse().matrix() << "\n";
		std::cout << ostrs.str() << std::endl;
	}
	//----------------------------------------------------------------------------
	// Apply transform matrix to source matrix
	// Note: SOURCE matrix can be read from .matrix or .ply file
	//----------------------------------------------------------------------------
	else if (action == "transform")
	{
		// affine transformation is 4 x 4 matrix
		Eigen::Affine3d matXform;
		if (!xformMatrix.empty() && File(xformMatrix).exists())
		{
			try
			{
				Eigen::MatrixXd matTrans = loadMatrix<double>(xformMatrix);
				if (matTrans.rows() == 3 && matTrans.cols() == 3)
					matXform.linear() = matTrans;
				else if (matTrans.rows() == 4 && matTrans.cols() == 4)
					matXform.matrix() = matTrans;
				else
					throw std::runtime_error("specified transform matrix has improper dimension");
			}
			catch (std::exception & e)
			{
				poco_error(logger(), string(e.what()));
				return Application::EXIT_DATAERR;
			}
		}
		else
		{
			if (xformMatrix.empty())
				poco_error(logger(), "/xformMatrix parameter is required to perform action transform");
			else
				poco_error(logger(), format("file %s does not exist", xformMatrix));
			return Application::EXIT_NOINPUT;
		}

		// convert build order string to all upper case
		std::transform(xformOrder.begin(), xformOrder.end(), xformOrder.begin(), [](char c) { return std::toupper(c, std::locale::classic()); });
		bool isXformLeft = (xformOrder != "IX") ? true : false;

		// load SOURCE and ANGLE matrix from .matrix or .ply
		Eigen::MatrixXd matSource;
		Eigen::MatrixXd matAngle;
		CloudMeta meta{};
		if (!sourceMatrix.empty() && File(sourceMatrix).exists())
		{
			try
			{
				// source should be m x 3 matrix, m points of (x, y, z) coordinates
				matSource = loadMatrix<double>(sourceMatrix);
				if (isXformLeft)
					// to 3 x m matrix
					matSource.transposeInPlace();

				if (!angleMatrix.empty() && File(angleMatrix).exists())
				{
					// angle should be m x 3 matrix, m points of (Rx, Ry, Rz) angles in degree
					matAngle = loadMatrix<double>(angleMatrix);
					// currently we use default ZYX form only
					meta.euler_angle = "zyx";
					if (isXformLeft)
						// to 3 x m matrix
						matAngle.transposeInPlace();
				}
			}
			catch (std::exception & e)
			{
				poco_error(logger(), string(e.what()));
				return Application::EXIT_DATAERR;
			}
		}
		else if (!sourcePly.empty() && File(sourcePly).exists())
		{
			vector<Vertex6d> v6dSource;
			try
			{
				v6dSource = PlyFile::readPlyToVertex6d(sourcePly, meta);
				Eigen::MatrixX3d source, angle;
				CloudConvert::splitFromVertex6d(v6dSource, source, angle);
				if (isXformLeft)
				{
					matSource = source.transpose();
					matAngle = angle.transpose();
				}
				else
				{
					matSource = source;
					matAngle = angle;
				}
			}
			catch (std::exception & e)
			{
				poco_error(logger(), string(e.what()));
				return Application::EXIT_DATAERR;
			}
		}
		else
		{
			poco_error(logger(), "SOURCE matrix argument is missing or invalid");
			return Application::EXIT_NOINPUT;
		}

		if (isXformLeft)
		{
			if (matSource.rows() != matXform.cols() - 1)
			{
				poco_error(logger(), "the dimension of source and transform matrix is not multipliable");
				return Application::EXIT_DATAERR;
			}
			// transform the coordinate matrix
			Eigen::MatrixXd resultMatrix = matXform * matSource.colwise().homogeneous();
			// output the matrix without homogeneous coefficients
			std::cout << "### Xform*SOURCE result:\n" << resultMatrix.block(0, 0, matSource.rows(), matSource.cols()) << "\n" << std::endl;

			if (matAngle.rows() != 3)
			{
				poco_error(logger(), "the dimension of angle matrix is not as expected");
				return Application::EXIT_DATAERR;
			}
			// transform the axis angles
			for (uint32_t i = 0; i < matAngle.cols(); ++i)
			{
				Eigen::Matrix3d xRotate(Eigen::AngleAxisd(matAngle.col(i).z() * EIGEN_PI / 180, Eigen::Vector3d::UnitZ())
					* Eigen::AngleAxisd(matAngle.col(i).y() * EIGEN_PI / 180, Eigen::Vector3d::UnitY())
					* Eigen::AngleAxisd(matAngle.col(i).x() * EIGEN_PI / 180, Eigen::Vector3d::UnitX()));
				xRotate = matXform.rotation() * xRotate;
				matAngle.col(i) = xRotate.eulerAngles(1, 0, 2) * 180 / EIGEN_PI;
			}
			std::cout << "### Xform.ROTATION*ANGLE result:\n" << matAngle << "\n" << std::endl;
		}
	}
	else
	{
		std::cout << "No action specified" << std::endl;
	}

	return Application::EXIT_OK;
}

bool AppTransform::helpRequested()
{
	return _helpRequested;
}
