#ifndef _LIDAR3D_PLUGIN_HH_
#define _LIDAR3D_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
    class lidar3D_plugin : public ModelPlugin
    {
        public: lidar3D_plugin(){}

        public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            std::cerr<<"\nThe 3D lidar plugin is attached to model ["<<_model->GetName()<<"]\n";
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(lidar3D_plugin)
}
#endif