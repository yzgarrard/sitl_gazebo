#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <string>

#define PRINT_VELOCITY_FROM_ROTORS true

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
            ignition::math::Vector3d distance = this->model->WorldPose().Pos() - this->drone->WorldPose().Pos();
            printf("YYYEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHH!\n");
            printf("Distance vector from drone to model:\n");
            printf("X:%f\tY:%f\n",
                   distance.X(),
                   distance.Y());
            // Listen to the update event. This event is broadcast every simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&PushedByDrone::OnUpdate, this));
        }

        // Called by the world update start event
    public:
        void OnUpdate() {
            auto velocityFromRotors = calculateVelocityDueToWindFromRotors();
            this->link->SetLinearVel(velocityFromRotors);
            counter++;
        }

    private:
        ignition::math::Vector3d calculateVelocityDueToWindFromRotors() {
            ignition::math::Vector3d distance = this->model->WorldPose().Pos() - this->drone->WorldPose().Pos();
            double windVelocityFromRotorsMagnitude;
            // When the target is a small distance away from the center of the drone, it will move from the wind. If the target is very close to
            // the center of the drone, it will behave as if there is no wind.
            if (std::sqrt(std::pow(distance.X(), 2) + std::pow(distance.Y(), 2)) >= 0.2) {
                windVelocityFromRotorsMagnitude = 12.13 * std::exp(-0.4334 * std::sqrt(std::pow(distance.X(), 2) + std::pow(distance.Y(), 2)));
            }
            else {
                windVelocityFromRotorsMagnitude = 0;
            }
            auto velocityFromRotors = ignition::math::Vector3d(
                    windVelocityFromRotorsMagnitude * std::cos(std::atan2(distance.Y(), distance.X())) * 0.035,
                    windVelocityFromRotorsMagnitude * std::sin(std::atan2(distance.Y(), distance.X())) * 0.035,
                    0
            );
            if (counter % 1000 == 0 && PRINT_VELOCITY_FROM_ROTORS) {
                printf("Velocity vector on model due to drone:\n");
                printf("X:%f\tY:%f\n",
                       velocityFromRotors.X(),
                       velocityFromRotors.Y());
            }
            return velocityFromRotors;
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