#pragma once

#include <array>
#include <fstream>
#include <initializer_list>
#include <any>

#include <Eigen/Eigen>

#include <geometry_msgs/Pose.h>

namespace Eigen
{

	// -- transformations ---------------------------------------------------------

	Eigen::Isometry3d
	make_tf(const std::array<double, 3>& pos, const Eigen::Vector3d& axis, double theta);

	// Eigen::Isometry3d // todo
	// make_tf(const std::array<double, 3>& pos, const std::array<double, 3>& axis, double theta);

	Eigen::Isometry3d
	make_tf(const std::array<double, 3>& pos, const std::array<double, 3>& rpy = {0, 0, 0});

	Eigen::Isometry3d
	make_tf(const geometry_msgs::Pose& pose);

	// -- operators---------------------------------------------------------------

	Eigen::MatrixXd
	pseudo_inverse(const Eigen::MatrixXd& mat, double lambda = 0.0);

	template<class Derived>
	inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
	skew(const Eigen::MatrixBase<Derived>& vec)
	{
		// https://github.com/ethz-asl/ethzasl_msf/blob/master/msf_core/include/msf_core/eigen_utils.h
		EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
		return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
	}

	// -- export -----------------------------------------------------------------

	struct ARGS_EXPORT_CSV {
		bool linewise_csv = false;
		bool row_major = false;
		std::_Ios_Openmode output_mode;
		Eigen::IOFormat format = Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
	};
	template <typename Derived>
	void
	export_csv(Eigen::MatrixBase<Derived>& matrix, const std::string& path, ARGS_EXPORT_CSV args = ARGS_EXPORT_CSV())
	{
		// write matrix to csv
		std::ofstream fs(path, args.output_mode);

		// prepare output matrix
		auto mat = matrix.derived();

		// output matrix as a row of text
		if (args.linewise_csv)
		{
			// transpose is row-major output desired
			if (args.row_major and not mat.IsRowMajor)
				mat.transposeInPlace();

			// map (view) matrix as row-vector
			Eigen::Map<Eigen::RowVectorXd> vec(mat.data(), mat.size());
			args.format = Eigen::IOFormat(StreamPrecision, DontAlignCols, ", ", ", "); // format for vectors
			fs << vec.format(args.format) << std::endl;
		}
		// output matrix as specified by Eigen::IOFormat
		else
		{
			fs << mat.format(args.format) << (args.output_mode == std::ofstream::app ? "\n" : "");
		}
	}

}