#include <gazebo/gazebo.hh>
#include <nos/print.h>
#include <nos/terminal.h>

#include <crow/gates/udpgate.h>
#include <crow/proto/pubsub.h>
#include <crow/address.h>
#include <crow/tower.h>

auto crowker = crow::address(".12.127.0.0.1:10009");

namespace gazebo
{
	class WorldPluginTutorial : public WorldPlugin
	{
	public:
		WorldPluginTutorial() : WorldPlugin()
		{
			printf("Hello World!\n");

			crow::create_udpgate(12);
			crow::start_spin();
		}

		void world_init() 
		{

			nos::reset_terminal();
			nos::println("world_init");

			crow::publish(crowker, "sim", "reset", 0, 0);

			nos::println("world_init .. ok");
		}

		virtual void Reset ()
		{
			world_init();
		}

	public:
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
		{
			world_init();
		}
	};
	GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}