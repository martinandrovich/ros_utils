#include "ros_utils/std.h"

#include <cstdio>
#include <ctime>

std::string
get_timestamp(const std::string& format)
{
	std::time_t rawtime;
	std::tm* timeinfo;
	char buffer [80];
	std::time(&rawtime);
	timeinfo = std::localtime(&rawtime);
	std::strftime(buffer, 80, format.c_str(), timeinfo);
	return std::string(buffer);
}