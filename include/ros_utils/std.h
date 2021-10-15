#include <initializer_list>
#include <algorithm>

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