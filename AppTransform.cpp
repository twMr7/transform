#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <type_traits>
#include <iterator>
#include <algorithm>
#include <limits>
#include <Poco/Util/Option.h>
#include <Poco/Util/HelpFormatter.h>
#include <Poco/Util/AbstractConfiguration.h>
#include <Poco/File.h>
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
using Poco::Util::Application;
using Poco::Util::Option;
using Poco::Util::OptionSet;
using Poco::Util::OptionCallback;
using Poco::Util::HelpFormatter;
using Poco::Util::AbstractConfiguration;
using Poco::File;

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
	optstrs << "(/@) specify the action to take for geometric transformation, available action types are:\n"
		<< "display - display information about points, angles, or transformation of matrix\n"
		<< "transform - apply TRANSFORM matrix on SOURCE matrix\n"
		<< "estimate - estimate transform matrix from SOURCE matrix to TARGET matrix";
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

	// This option assign argument directly to configuration "application.target.ply".
	options.addOption(
		Option("goalPly", "g", "(/g) TARGET data from .ply file")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.target.ply"));

	// This option assign argument directly to configuration "application.transform".
	options.addOption(
		Option("transMatrix", "t", "(/t) TRANSFORM matrix from .matrix file")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.transform.matrix"));

	// This option assign argument directly to configuration "application.rotation".
	options.addOption(
		Option("rotation", "r", "(/r) ROTATION matrix from .matrix file")
		.required(false)
		.repeatable(false)
		.argument("file")
		.binding("application.rotation.matrix"));

	// This option assign argument directly to configuration "application.scaling".
	options.addOption(
		Option("scaling", "s", "(/s) set scaling factor, or set scaling or not when estimate transform matrix")
		.required(false)
		.repeatable(false)
		.argument("factor")
		.binding("application.scaling"));
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> loadMatrix(string filename)
{
	static_assert(std::is_floating_point<T>::value, "only floating point types are supported");

	using MatrixType = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

	if (File(filename).exists())
	{
		ifstream fin(filename, std::ios::in | std::ios::binary);
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
	else
	{
		return -1 + MatrixType::Zero(1, 1);
	}
}

int AppTransform::main(const ArgVec & args)
{
	if (_helpRequested)
		return Application::EXIT_USAGE;

	string action = config().getString("application.action", "");
	string sourceMatrix = config().getString("application.source.matrix", "");
	string sourcePly = config().getString("application.source.ply", "");
	string targetMatrix = config().getString("application.target.matrix", "");
	string targetPly = config().getString("application.target.ply", "");
	string transMatrix = config().getString("application.transform.matrix", "");

	if (action == "display")
	{
		if (!transMatrix.empty())
		{
			Eigen::MatrixXd matTrans = loadMatrix<double>(transMatrix);
			Eigen::Transform<double, 4, Eigen::Affine> t4(matTrans);
			std::cout << "--- Transform Matrix:\n" << t4.matrix() << std::endl;
		}
	}

#if 0
	//----------------------------------------------------------------------------
	// Transform matrix estimation using umeyama
	//----------------------------------------------------------------------------
	string estimateTarget = config().getString("application.estimatetarget", "");
	if (!estimateTarget.empty())
	{
		poco_information(logger(), "\n===== Transformation Matrix Estimation using Umeyama =====");
		try
		{
			Eigen::MatrixXd targetMatrix = loadMatrix<double>(estimateTarget);
			targetMatrix.transposeInPlace();

			Eigen::MatrixXd sourceMatrix = loadMatrix<double>(config().getString("application.source", ""));
			sourceMatrix.transposeInPlace();

			std::cout << "Source Matrix:\n" << sourceMatrix << std::endl;
			std::cout << "Destination Matrix:\n" << targetMatrix << std::endl;

			if (sourceMatrix.rows() != targetMatrix.rows() || sourceMatrix.cols() != targetMatrix.cols())
			{
				poco_error(logger(), "the dimension of source and destination matrix must match");
				return Application::EXIT_DATAERR;
			}

			if (sourceMatrix.rows() < 2 || sourceMatrix.cols() < 2)
			{
				poco_error(logger(), "matrix must at least 2x2");
				return Application::EXIT_DATAERR;
			}

			bool scaling = config().getBool("application.scaling", false);
			Eigen::MatrixXd cR_t = Eigen::umeyama(sourceMatrix, targetMatrix, scaling);
			double error = (cR_t * sourceMatrix.colwise().homogeneous() - targetMatrix.colwise().homogeneous()).squaredNorm();
			std::cout << "Transform Matrix:\n" << cR_t << std::endl;
			//std::cout << "Transform Result:\n" << cR_t * sourceMatrix.colwise().homogeneous() << std::endl;
			std::cout << "estimation error = " << error << std::endl;
			std::cout << "where epsilon is " << std::numeric_limits<double>::epsilon() << "\n" << std::endl;
		}
		catch (std::exception & e)
		{
			poco_error(logger(), string(e.what()));
		}
	}

	//----------------------------------------------------------------------------
	// Apply transform matrix to source matrix
	//----------------------------------------------------------------------------
	string transform = config().getString("application.transform", "");
	if (!transform.empty())
	{
		poco_information(logger(), "\n===== Apply Transformation to Source Matrix  =====");
		try
		{
			Eigen::MatrixXd transformMatrix = loadMatrix<double>(transform, 4);

			Eigen::MatrixXd sourceMatrix = loadMatrix<double>(config().getString("application.source", ""));
			sourceMatrix.transposeInPlace();

			std::cout << "Source Matrix:\n" << sourceMatrix << std::endl;
			std::cout << "Transform Matrix:\n" << transformMatrix << std::endl;

			if (sourceMatrix.rows() != transformMatrix.cols() - 1)
			{
				poco_error(logger(), "the dimension of source and transform matrix is not multipliable");
				return Application::EXIT_DATAERR;
			}

			Eigen::MatrixXd resultMatrix = transformMatrix * sourceMatrix.colwise().homogeneous();
			// output the matrix without homogeneous coefficients
			std::cout << "Transform result:\n" << resultMatrix.block(0, 0, sourceMatrix.rows(), sourceMatrix.cols()) << "\n" << std::endl;
		}
		catch (std::exception & e)
		{
			poco_error(logger(), string(e.what()));
		}
	}
#endif

	return Application::EXIT_OK;
}

bool AppTransform::helpRequested()
{
	return _helpRequested;
}
