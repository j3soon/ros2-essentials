/**
 * @file /include/ecl/sigslots_lite/managers.hpp
 *
 * @brief Management classes for sigslots lite.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_LITE_MANAGERS_HPP_
#define ECL_SIGSLOTS_LITE_MANAGERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <array>

#include "slot.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace lite {

/*****************************************************************************
** Forward Declarations
*****************************************************************************/

template <typename Data, unsigned int Capacity> class Signal;

/*****************************************************************************
** Managers
*****************************************************************************/

namespace sigslots {
/**
 * @brief Used internally to retrieve info about members lots.
 */
template <typename Data, typename FunctionClass>
class MemberSlotsBase {
public:
	virtual unsigned int stored() const { return 0; }
	virtual unsigned int capacity() const { return 0; }
	virtual sigslots::MemberSlot<Data,FunctionClass>* addSlot(void (FunctionClass::*func)(Data), FunctionClass &instance) = 0;
};

/**
 * @brief Used internally to retrieve info about void member slots.
 */
template <typename FunctionClass>
class MemberSlotsBase<void,FunctionClass> {
public:
	virtual unsigned int stored() const { return 0; }
	virtual unsigned int capacity() const { return 0; }
	virtual sigslots::MemberSlot<void,FunctionClass>* addSlot(void (FunctionClass::*func)(), FunctionClass &instance) = 0;
};

template <typename Data, typename GlobalClass>
class GlobalSlotsBase {
public:
	virtual unsigned int stored() const { return 0; }
	virtual unsigned int capacity() const { return 0; }
	virtual sigslots::GlobalSlot<Data>* addSlot( void (*function)(Data) ) = 0;
};

template <typename GlobalClass>
class GlobalSlotsBase<void, GlobalClass> {
public:
	virtual unsigned int stored() const { return 0; }
	virtual unsigned int capacity() const { return 0; }
	virtual sigslots::GlobalSlot<void>* addSlot( void (*function)() ) = 0;
};

} // namespace sigslots

/******************************************
** Member Manager
*******************************************/
/**
 * @brief This is the member slot interface, inheritable by classes.
 *
 * To enable slots for your class, simply inherit this interface.
 *
 * @code
 * class Foo : public ecl::lite::MemberSlots<const char*,Foo> {
 * public:
 * 	   void f(const char* str) {
 * 		   std::cout << "Foo::f() " << str << std::endl;
 * 	   }
 * };
 *
 * int main() {
 *     Foo foo;
 *     ecl::lite::Signal<const char*> signal;
 *     connect(signal,&Foo::f,foo);
 *     signal.emit("Dude");
 * @endcode
 *
 * @tparam Data : the footprint for the member function to slot.
 * @tparam FunctionClass : the class it will provide slots for.
 * @tparam Capacity : the number of functions from FunctionClass you'll slot.
 */
template <typename Data, typename FunctionClass, unsigned int Capacity = 1>
class MemberSlots : public sigslots::MemberSlotsBase<Data,FunctionClass> {
public:
	/*********************
	** Friends
	**********************/
	// allow connect to use addSlot
	template <typename Data_, unsigned int Capacity_, typename FunctionClass_>
	friend sigslots::Error connect(Signal<Data_,Capacity_> &signal, void(FunctionClass_::*f)(Data_), FunctionClass_ &o);

	// needs access to stored()
	template <typename Data_, typename FunctionClass_>
	friend unsigned int member_slots_stored(const FunctionClass_ &object);

	// needs access to capacity()
	template <typename Data_, typename FunctionClass_>
	friend unsigned int member_slots_capacity(const FunctionClass_ &object);


protected:
	MemberSlots() : size(0) {};

private:
	/**
	 * @brief The number of slots stored.
	 *
	 * @return unsigned int : storage size.
	 */
	unsigned int stored() const { return size; }
	/**
	 * @brief The number of slots that can be attached to member functions.
	 *
	 * @return unsigned int : the maximum capacity.
	 */
	unsigned int capacity() const { return Capacity; }

	/**
	 * @brief Add a slot.
	 *
	 * This is used 'under the hood' by the connectors.
	 *
	 * @param func : the function to slot.
	 * @param instance : the class instance associated with the function.
	 * @return MemberSlot* : a pointer to a member slot.
	 */
	sigslots::MemberSlot<Data,FunctionClass>* addSlot(void (FunctionClass::*func)(Data), FunctionClass &instance) {
		for ( unsigned int i = 0; i < size; ++i ) {
			if ( func == slots[i].member_function ) {
				return &(slots[i]);
			}
		}
		if ( size < Capacity ) {
			slots[size] = sigslots::MemberSlot<Data,FunctionClass>(func,instance);
			++size;
			return &(slots[size-1]);
		} else {
			return NULL;
		}
	}

	unsigned int size;
	sigslots::MemberSlot<Data,FunctionClass> slots[Capacity];
};

/******************************************
** Global Manager
*******************************************/

template <typename Data, typename GlobalClass, unsigned int Capacity = 1>
class GlobalSlots : public sigslots::GlobalSlotsBase<Data, GlobalClass> {
public:
	/*********************
	** Friends
	**********************/
	// allow connect to use addSlot
	template <typename Data_, unsigned int Capacity_, typename GlobalClass_>
	friend sigslots::Error connect(Signal<Data_,Capacity_> &signal, void(*function)(Data_), GlobalClass_ &g);

	// needs access to stored()
	template <typename Data_, typename GlobalClass_>
	friend unsigned int global_slots_stored();

	// needs access to capacity()
	template <typename Data_, typename GlobalClass_>
	friend unsigned int global_slots_capacity();

protected:
	GlobalSlots() : size(0) {};

private:
	/**
	 * @brief The number of slots stored.
	 *
	 * @return unsigned int : storage size.
	 */
	unsigned int stored() const { return size; }
	/**
	 * @brief The number of slots that can be attached to member functions.
	 *
	 * @return unsigned int : the maximum capacity.
	 */
	unsigned int capacity() const { return Capacity; }

	/**
	 * @brief Add a slot.
	 *
	 * This is used 'under the hood' by the connectors.
	 *
	 * @param func : the function to slot.
	 * @param instance : the class instance associated with the function.
	 * @return MemberSlot* : a pointer to a member slot.
	 */
	sigslots::GlobalSlot<Data>* addSlot(void (*function)(Data)) {
		for ( unsigned int i = 0; i < size; ++i ) {
			if ( function == slots[i].global_function ) {
				return &(slots[i]);
			}
		}
		if ( size < Capacity ) {
			slots[size] = sigslots::GlobalSlot<Data>(function);
			++size;
			return &(slots[size-1]);
		} else {
			return NULL;
		}
	}

	unsigned int size;
	sigslots::GlobalSlot<Data> slots[Capacity];
};

/*****************************************************************************
** Specialisations
*****************************************************************************/
/**
 * @brief Specialisation for void member slots management.
 */
template <typename FunctionClass, unsigned int Capacity>
class MemberSlots<void, FunctionClass, Capacity> : public sigslots::MemberSlotsBase<void,FunctionClass> {
public:
	/*********************
	** Friends
	**********************/
	// allow connect to use addSlot
	template <unsigned int Capacity_, typename FunctionClass_>
	friend sigslots::Error connect(Signal<void,Capacity_> &signal, void(FunctionClass_::*f)(void), FunctionClass_ &o);
	// needs access to stored()
	template <typename Data_, typename FunctionClass_>
	friend unsigned int member_slots_stored(const FunctionClass_ &object);

	// needs access to capacity()
	template <typename Data_, typename FunctionClass_>
	friend unsigned int member_slots_capacity(const FunctionClass_ &object);

protected:
	MemberSlots() : size(0) {};

private:
	/**
	 * @brief The number of slots stored.
	 *
	 * @return unsigned int : storage size.
	 */
	unsigned int stored() const { return size; }

	/**
	 * @brief The number of slots that can be attached to member functions.
	 *
	 * @return unsigned int : the maximum capacity.
	 */
	unsigned int capacity() const { return Capacity; }

	/**
	 * @brief Add a slot.
	 *
	 * This is used 'under the hood' by the connectors.
	 *
	 * @param func : the function to slot.
	 * @param instance : the class instance associated with the function.
	 * @return MemberSlot* : a pointer to a member slot.
	 */
	sigslots::MemberSlot<void,FunctionClass>* addSlot(void (FunctionClass::*func)(void), FunctionClass &instance) {
		for ( unsigned int i = 0; i < size; ++i ) {
			if ( func == slots[i].member_function ) {
				return &(slots[i]);
			}
		}
		if ( size < Capacity ) {
			slots[size] = sigslots::MemberSlot<void,FunctionClass>(func,instance);
			++size;
			return &(slots[size-1]);
		} else {
			return NULL;
		}
	}

	unsigned int size;
	sigslots::MemberSlot<void,FunctionClass> slots[Capacity];
};

template <typename GlobalClass, unsigned int Capacity>
class GlobalSlots<void, GlobalClass, Capacity> : public sigslots::GlobalSlotsBase<void, GlobalClass> {
public:
	/*********************
	** Friends
	**********************/
	// allow connect to use addSlot
	template <unsigned int Capacity_, typename GlobalClass_>
	friend sigslots::Error connect(Signal<void,Capacity_> &signal, void(*function)(void), GlobalClass_ &g);

	// needs access to stored()
	template <typename Data_, typename GlobalClass_>
	friend unsigned int global_slots_stored();

	// needs access to capacity()
	template <typename Data_, typename GlobalClass_>
	friend unsigned int global_slots_capacity();

protected:
	GlobalSlots() : size(0) {};

private:
	/**
	 * @brief The number of slots stored.
	 *
	 * @return unsigned int : storage size.
	 */
	unsigned int stored() const { return size; }
	/**
	 * @brief The number of slots that can be attached to member functions.
	 *
	 * @return unsigned int : the maximum capacity.
	 */
	unsigned int capacity() const { return Capacity; }

	/**
	 * @brief Add a slot.
	 *
	 * This is used 'under the hood' by the connectors.
	 *
	 * @param func : the function to slot.
	 * @param instance : the class instance associated with the function.
	 * @return GlobalSlot* : a pointer to a member slot.
	 */
	sigslots::GlobalSlot<void>* addSlot(void (*function)()) {
		for ( unsigned int i = 0; i < size; ++i ) {
			if ( function == slots[i].global_function ) {
				return &(slots[i]);
			}
		}
		if ( size < Capacity ) {
			slots[size] = sigslots::GlobalSlot<void>(function);
			++size;
			return &(slots[size-1]);
		} else {
			return NULL;
		}
	}

	unsigned int size;
	sigslots::GlobalSlot<void> slots[Capacity];
};


class Joe {
public:
    static inline void addSlot() {
        // static std::array<float, GlobalSlots<void>::capacity> dods;
        static std::array<float, 5> dods;
        dods[0] = 3.0;
        // std::cout << "Capacity: " << GlobalSlots<void>::capacity << std::endl;
        std::cout << dods[0] << std::endl;
    }
};


//template <typename Data, unsigned int Capacity>
//struct GlobalManager : public GlobalSlots<Data, GlobalManager, Capacity> {};

// template <typename Data, unsigned int Capacity = 1>
// const GlobalManager<Data>& global_manager() {
//     static GlobalManager<Data> god{};
//     return god;
// }

//template <>
//const GlobalManager<void>& global_manager() {
//    static GlobalManager<void> god{};
//    return god;
//}


} // namespace lite
} // namespace ecl

#endif /* ECL_SIGSLOTS_LITE_MANAGERS_HPP_ */
