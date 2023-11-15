/**
 * @file /ecl_sigslots_lite/include/ecl/sigslots_lite/utilities.hpp
 *
 * @brief Utility functions
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_LITE_UTILITY_HPP_
#define ECL_SIGSLOTS_LITE_UTILITY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "managers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace lite {

template <typename Data, typename GlobalClass>
unsigned int global_slots_stored(const GlobalClass &object) {
    const sigslots::GlobalSlotsBase<Data,GlobalClass> &global_slots = object;
    return global_slots.stored();
}

template <typename Data, typename GlobalClass>
unsigned int global_slots_capacity(const GlobalClass &object) {
    const sigslots::GlobalSlotsBase<Data,GlobalClass> &global_slots = object;
    return global_slots.capacity();
}

template <typename Data, typename FunctionClass>
unsigned int member_slots_stored(const FunctionClass &object) {
	const sigslots::MemberSlotsBase<Data,FunctionClass> &member_slots = object;
	return member_slots.stored();
}

template <typename Data, typename FunctionClass>
unsigned int member_slots_capacity(const FunctionClass &object) {
	const sigslots::MemberSlotsBase<Data,FunctionClass> &member_slots = object;
	return member_slots.capacity();
}

} // namespace lite
} // namespace ecl

#endif /* ECL_SIGSLOTS_LITE_UTILITY_HPP_ */
