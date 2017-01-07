#include <gazebo/gazebo.hh>

namespace gazebo {

	/*Each plugin must inherit from a puglin type: WorldPlugin class*/
	class WorldPluginTutorial : public WorldPlugin {
		public: WorldPluginTutorial() : WorldPlugin(){
			printf("Hello World!\n");
		}

		/*Load fn is mandatory; recieves SDF element that contains the elements & attributes specified in loaded SDF file*/
		public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

		}
	};

	/*Plugin must be register w/ simulation using GZ_REGISTER_WORLD_PLUGIN macro*/
	GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}