#include <math.h>
#include <type_traits>

namespace math
{

	template<typename T>
	T
	normalize_angle(T angle)
	{
		static_assert(std::is_floating_point<T>::value, "normalize_angle(T) only works for floating point types.");
		
		while (angle > M_PI)
			angle -= 2*M_PI;
			
		while (angle < -M_PI)
			angle += 2*M_PI;
		
		return angle;
	}

}