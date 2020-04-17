#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <string>

#define PRINT_VELOCITY_FROM_ROTORS true

namespace gazebo {
    class FPVCamOneInit : public ModelPlugin {

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            // Store the pointer to the model
            this->model = _parent;
            this->link = this->model->GetChildLink("link");
            this->worldPtr = this->model->GetWorld();
			auto position = this->model->WorldPose().Pos();
			printf("Initial position of camera_1: X:%f\tY:%f\tZ:%f\n", position.X(), position.Y(), position.Z());			
			
			// Get pointers to drones
			this->drone_1 = this->worldPtr->ModelByName("iris_1");
			this->drone_link = this->drone_1->GetChildLink("base_link");
			auto drone_1_pos = this->drone_1->WorldPose().Pos();
			printf("Position of iris_1: X:%f\tY:%f\tZ:%f\n", drone_1_pos.X(), drone_1_pos.Y(), drone_1_pos.Z());
		
			// Move the cameras to the center of the drones
			ignition::math::Pose3d camera_1_pos = ignition::math::Pose3d(drone_1_pos.X(), drone_1_pos.Y(), drone_1_pos.Z(), 0, 3.15159/2.0, 0);
			this->model->SetWorldPose(camera_1_pos);
			this->model->CreateJoint("cam_1_joint", "fixed", drone_link, link);
        }

        // Called by the world update start event
    public:
        // void OnUpdate() {

        // }

        // Pointer to the model
    private:
		bool cameras_set = false;
        physics::WorldPtr worldPtr;
        physics::ModelPtr model;
        physics::LinkPtr link;
		physics::LinkPtr drone_link;
        physics::ModelPtr camera_1;
		physics::ModelPtr drone_1;
        long long counter = 0;


        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(FPVCamOneInit)
}