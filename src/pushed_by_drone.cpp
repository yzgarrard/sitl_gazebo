#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <string>

namespace gazebo {
    class PushedByDrone : public ModelPlugin {

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            // Store the pointer to the model
            this->model = _parent;
            this->link = this->model->GetChildLink("link");
            this->worldPtr = this->model->GetWorld();
            this->drone = this->worldPtr->ModelByName("iris_downward_cam");
            physics::Joint_V joints = this->drone->GetJoints();
            printf("YYYEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHH!\n");
            int rotors_idx = 0;
            for (auto & joint : joints) {
                std::string str = joint->GetName();
                if (str.find("rotor_") != -1) {
                    this->droneRotors.push_back(joint);
                    ignition::math::Vector3d worldPos = joint->WorldPose().Pos();
                    printf("%f\t%f\t%f\n", worldPos.X(), worldPos.Y(), worldPos.Z());
                }
            }
            printf("Vector from each rotor to model:\n");
            for (int i = 0; i < this->droneRotors.size(); i++) {
                ignition::math::Vector3d distance = this->model->WorldPose().Pos() - this->droneRotors[i]->WorldPose().Pos();
                printf("%d:\tX:%f\tY:%f\tZ:%f\n", i, distance.X(), distance.Y(), distance.Z());
            }
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&PushedByDrone::OnUpdate, this));
        }

        // Called by the world update start event
    public:
        void OnUpdate() {
            this->link->SetForce(ignition::math::Vector3d(0, 0, 0));    // Clears previously set force
            ignition::math::Vector3d newForce = ignition::math::Vector3d(0, 0, 0);
            // Add forces based on distance to rotors
            ignition::math::Vector3d relativePosition = this->model->WorldPose().Pos() - this->drone->WorldPose().Pos();
            for (int i = 0; i < this->droneRotors.size(); i++) {
                ignition::math::Vector3d distance = this->model->WorldPose().Pos() - this->droneRotors[i]->WorldPose().Pos();
                printf("%d:\tX:%f\tY:%f\tFx:%f\tFy:%f\n",
                        i,
                        distance.X(),
                        distance.Y(),
                        std::min(1/(std::sqrt(std::pow(distance.X(), 2) + std::pow(distance.Z(), 2))), 5.0) * ignition::math::signum(distance.X()),
                        std::min(1/(std::sqrt(std::pow(distance.Y(), 2) + std::pow(distance.Z(), 2))), 5.0) * ignition::math::signum(distance.Y()));
                newForce = newForce + ignition::math::Vector3d(
                        std::min(1/(std::sqrt(std::pow(distance.X(), 2) + std::pow(distance.Z(), 2))), 5.0) * ignition::math::signum(distance.X()),
                        std::min(1/(std::sqrt(std::pow(distance.Y(), 2) + std::pow(distance.Z(), 2))), 5.0) * ignition::math::signum(distance.Y()),
                        0);    //idk if division by zero will happen
            }
            this->link->AddForce(newForce);
            printf("Sum:\tFx:%f\tFy:%f\n", newForce.X(), newForce.Y());
            if (counter % 1000 == 0) {
                printf("Vector from each rotor to model:\n");
                printf("Velocity of model:\tX:%50.50f\tY:%50.50f\tZ:%50.50f\n", this->model->WorldLinearVel().X(), this->model->WorldLinearVel().Y(), this->model->WorldLinearVel().Z());
                printf("Position of model:\tX:%50.50f\tY:%50.50f\tZ:%50.50f\n", this->model->WorldPose().Pos().X(), this->model->WorldPose().Pos().Y(), this->model->WorldPose().Pos().Z());
            }
            // Apply a small linear velocity to the model.
            // this->model->SetLinearVel(ignition::math::Vector3d(1-relativePosition.X(), 1-relativePosition.Y(), 1-relativePosition.Z()));
            counter++;
        }

        // Pointer to the model
    private:
        physics::WorldPtr worldPtr;
        physics::ModelPtr model;
        physics::LinkPtr link;
        physics::ModelPtr drone;
        physics::Joint_V droneRotors;
        long long counter = 0;


        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PushedByDrone)
}