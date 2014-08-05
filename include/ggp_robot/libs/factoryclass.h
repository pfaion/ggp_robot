#ifndef __GGP_ROBOT_FACTORYCLASS_H
#define __GGP_ROBOT_FACTORYCLASS_H

#include <map>
#include <string>

/** Factory for derived type registration. Registration is done with macro:
  REGISTER_TYPE(PARENT_TYPE, TYPE, STRING)

  Example-usage: registering a derived class DerivedA from base class Base with
  REGISTER_TYPE(Base, DerivedA, "myA");
  allows to call
  Base* b = Factory<Base>::create("myA");
  which returns a Base-Pointer to an instance of the derived class DerivedA;

  Code is mostly taken from:
  http://stackoverflow.com/questions/6137706#6144705 */
template<class T>
struct Factory {

  public:
    /** Creates the object from the corresponding string */
    static T* create(const std::string& id) {
      const typename Creators_t::const_iterator iter = static_creators().find(id);
      return iter == static_creators().end() ? 0 : (*iter->second)();
    }

    typedef T* Creator_t();
    typedef std::map<std::string, Creator_t*> Creators_t;

    /** Get a static instance of the map */
    static Creators_t& static_creators() {
      static Creators_t s_creators;
      return s_creators;
    }

    /** Inner class for registering types */
    template<class T2 = int> struct Register {
      static T* create() { return new T2(); };
      static Creator_t* init_creator(const std::string& id) {
        return static_creators()[id] = create;
      }
      static Creator_t* creator;
    };
};

/** Definition of registration macro */
#define REGISTER_TYPE(P, T, STR)                         \
  template<> template<> Factory<P>::Creator_t*           \
  Factory<P>::Register<T>::creator =          \
  Factory<P>::Register<T>::init_creator(STR)

# endif

