#ifndef __GGP_ROBOT_FACTORIES_H
#define __GGP_ROBOT_FACTORIES_H

#include <map>
#include <string>

namespace GgpRobot {
  
  /** Factory for Planar Boards. Registration is done with macro:
      REGISTER_TYPE(TYPE, STRING) */
  struct PlanarBoardFactory {

    public:
      /** Creates the object from the corresponding string */
      static PlanarBoard* create(const std::string& id) {
        const Creators_t::const_iterator iter = static_creators().find(id);
        return iter == static_creators().end() ? 0 : (*iter->second)();
      }

    private:
      /** Typedefs make life easier */
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

}
/** Definition of registration macro */
#define REGISTER_TYPE(T, STR)                                   \
  template<> GgpRobot::PlanarBoardFactory::Creator_t*           \
  GgpRobot::PlanarBoardFactory::Register<T>::creator =          \
  GgpRobot::PlanarBoardFactory::Register<T>::init_creator(STR)

# endif
