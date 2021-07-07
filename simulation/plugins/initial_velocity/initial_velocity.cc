#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->x_vel = _sdf->GetElement("x_velocity")->Get<double>();
      this->y_vel = _sdf->GetElement("y_velocity")->Get<double>();
      this->z_vel = _sdf->GetElement("z_velocity")->Get<double>();
      this->model->SetLinearVel(ignition::math::Vector3d(this->x_vel, this->y_vel, this->z_vel));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: double x_vel;
    private: double y_vel;
    private: double z_vel;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}