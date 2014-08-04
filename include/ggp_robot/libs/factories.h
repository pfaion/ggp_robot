#ifndef __GGP_ROBOT_FACTORIES_H
#define __GGP_ROBOT_FACTORIES_H

#include <map>
#include <string>

class PlanarBoard;

/** Factory for planar boards. Registration is done with macro:
  REGISTER_TYPE(TYPE, STRING)

  Code is taken from:
http://stackoverflow.com/questions/6137706#6144705 */
struct PlanarBoardFactory {

  public:
    /** Creates the object from the corresponding string */
    static PlanarBoard* create(const std::string& id) {
      const Creators_t::const_iterator iter = static_creators().find(id);
      return iter == static_creators().end() ? 0 : (*iter->second)();
    }

    typedef PlanarBoard* Creator_t();
    typedef std::map<std::string, Creator_t*> Creators_t;

    /** Get a static instance of the map */
    static Creators_t& static_creators() {
      static Creators_t s_creators;
      return s_creators;
    }

    /** Inner class for registering types */
    template<class T = int> struct Register {
      static PlanarBoard* create() { return new T(); };
      static Creator_t* init_creator(const std::string& id) {
        return static_creators()[id] = create;
      }
      static Creator_t* creator;
    };
};

/** Definition of registration macro */
#define REGISTER_TYPE(T, STR)                         \
  template<> PlanarBoardFactory::Creator_t*           \
  PlanarBoardFactory::Register<T>::creator =          \
  PlanarBoardFactory::Register<T>::init_creator(STR)

/** Registration of necessary objects is done in separate file */
#include <ggp_robot/libs/factory_includes.h>


# endif

