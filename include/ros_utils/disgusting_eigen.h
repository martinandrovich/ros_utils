// optional arguments for export_csv()
	struct EXPORT_CSV_LINEWISE{}; struct EXPORT_CSV_APPEND{};

	template <typename Derived, typename... Args>
	void
	export_csv(Eigen::MatrixBase<Derived>& matrix, const std::string& path, Args... args)
	{
		// params (default values)
		auto linewise_csv = false;
		auto format_csv = Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
		auto open_mode = std::ofstream::out;

		// compose a list of arguments from the variadic parameter pack 'Args... args'
		for (auto arg : std::initializer_list<std::any>{args...})
		{
			if (arg.type() == typeid(EXPORT_CSV_LINEWISE))
			{
				linewise_csv = true;
				format_csv = Eigen::IOFormat(StreamPrecision, DontAlignCols, ", ", ", "); // format for vectors
			}

			if (arg.type() == typeid(EXPORT_CSV_APPEND))
				open_mode = std::ofstream::app;

			if (arg.type() == typeid(Eigen::IOFormat))
				format_csv = std::any_cast<Eigen::IOFormat>(arg);
		}

		// write matrix to csv
		std::ofstream fs(path, open_mode);

		if (linewise_csv)
		{
			// map (view) matrix as row-vector (matrix is still column-major storage)
			Eigen::Map<Eigen::RowVectorXd> vec(matrix.derived().data(), matrix.derived().size());
			fs << vec.format(format_csv) << std::endl;
		}
		else
		{
			fs << matrix.format(format_csv);
		}
	}