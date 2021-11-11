#pragma once

#include <iostream>
#include <string>
#include <initializer_list>
#include <algorithm>
#include <memory>

std::string // todo
get_timestamp(const std::string& format = "%Y%m%d_%H%M%S");

inline void
ENTER_TO_CONTINUE(const std::string& msg = "continue")
{
	std::cout << "\nPress [ENTER] to " << msg << "..."; std::cin.ignore();
}

// -- cout overloads for container (array, vector) ----------------------------
// currently only implemented for vector; can be made better with C++20 concepts

template<typename T>
std::ostream&
operator << (std::ostream& os, const std::vector<T>& vec)
{
	os << "[";
	for (auto i = 0; i < vec.size(); i++)
		os << vec[i] << (i < (vec.size() - 1) ? ", " : "");
	os << "]";
	return os;
}

// https://stackoverflow.com/a/51532253/1658105
// template<typename T, template<typename, typename> class Container>
// std::ostream&
// operator << (std::ostream& os, const Container<T, std::allocator<T>>& c)
// {
// 	os << "[";
// 	for (auto i = 0; i < c.size(); i++)
// 		os << c[i] << (i < (c.size() - 1) ? ", " : "");
// 	os << "]";
// 	return os;
// }

// -- is_in() -----------------------------------------------------------------

template<class T>
struct type_identity { using type = T; };

template<class T>
using type_identity_t = typename type_identity<T>::type;

template<typename Element, typename Container>
bool is_in(const Element& e, const Container& c)
{
	// https://stackoverflow.com/questions/20303821/how-to-check-if-string-is-in-array-of-strings
	return std::find(std::begin(c), std::end(c), e) != std::end(c);
}
	
template<typename Element>
bool is_in(Element e, std::initializer_list<type_identity_t<Element>> l)
{
	// https://stackoverflow.com/a/69581543/1658105
	// https://en.cppreference.com/w/cpp/types/type_identity#Possible_implementation
	return is_in<Element, std::initializer_list<Element>>(e, l);
}