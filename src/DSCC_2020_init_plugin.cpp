#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <string>
#include "../include/zhelpers.hpp"

#define PRINT_VELOCITY_FROM_ROTORS true

namespace gazebo {
    class DSCC2020Init : public ModelPlugin {

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            // Store the pointer to the model
            model = _parent;
            link = model->GetChildLink("beam");
            red_attachment_point = model->GetChildLink("red_attachment_point");
            green_attachment_point = model->GetChildLink("green_attachment_point");
            blue_attachment_point = model->GetChildLink("blue_attachment_point");
            CoM = model->GetChildLink("CoM");
            worldPtr = model->GetWorld();
			auto position = model->WorldPose().Pos();
			printf("Initial position of beam: X:%f\tY:%f\tZ:%f\n", position.X(), position.Y(), position.Z());
			printf("Setting position of beam to: X:%f\tY:%f\tZ:%f\n", 0.0,  2.5, 0.05);
			ignition::math::Pose3d newPosition = ignition::math::Pose3d(1.0,  2.5, 0.05, 0.0, 0.0, 0.0);
			model->SetWorldPose(newPosition);
			
			// Insert cameras
			worldPtr->InsertModelFile("model://fpv_cam_0");
			worldPtr->InsertModelFile("model://fpv_cam_1");

			// Get pointers to drones
			drone_0 = worldPtr->ModelByName("iris_0");
			drone_1 = worldPtr->ModelByName("iris_1");
			drone_0_link = drone_0->GetChildLink("base_link");
            drone_1_link = drone_1->GetChildLink("base_link");
			auto drone_0_pos = drone_0->WorldPose().Pos();
			auto drone_1_pos = drone_1->WorldPose().Pos();
			printf("Position of iris_0: X:%f\tY:%f\tZ:%f\n", drone_0_pos.X(), drone_0_pos.Y(), drone_0_pos.Z());
			printf("Position of iris_1: X:%f\tY:%f\tZ:%f\n", drone_1_pos.X(), drone_1_pos.Y(), drone_1_pos.Z());

            //  Prepare our context and subscriber
            context = zmq::context_t(1);
            subscriber = zmq::socket_t(context, ZMQ_SUB);
            subscriber.connect("tcp://localhost:5563");
            subscriber.setsockopt( ZMQ_SUBSCRIBE, "joint*", 1);

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&DSCC2020Init::OnUpdate, this));
        }

    public:
        void OnUpdate() {
            //  Read envelope with address
            std::string address = s_recv (subscriber, ZMQ_DONTWAIT);
            //  Read message contents
            std::string contents = s_recv (subscriber, ZMQ_DONTWAIT);

            if (!contents.empty()) {
                printf("Address: %s\tContent: %s\n", address.c_str(), contents.c_str() );
                if (address == "joint0") {
                    if (contents == "red_attachment_point" && !joint0enabled) {
                        joint0 = model->CreateJoint("joint0", "fixed", drone_0_link, red_attachment_point);
                        joint0enabled = true;
                        printf("Created fixed joint between drone0 and red attachment point\n");
                    }
                    else if (contents == "green_attachment_point" && !joint0enabled) {
                        joint0 = model->CreateJoint("joint0", "fixed", drone_0_link, green_attachment_point);
                        joint0enabled = true;
                        printf("Created fixed joint between drone0 and green attachment point\n");
                    }
                    else if (contents == "blue_attachment_point" && !joint0enabled) {
                        joint0 = model->CreateJoint("joint0", "fixed", drone_0_link, blue_attachment_point);
                        joint0enabled = true;
                        printf("Created fixed joint between drone0 and blue attachment point\n");
                    }
                    else if (contents == "beam" && !joint0enabled) {
                        joint0 = model->CreateJoint("joint0", "fixed", drone_0_link, link);
                        joint0enabled = true;
                        printf("Created fixed joint between drone0 and beam\n");
                    }
                    else if (contents == "CoM" && !joint0enabled) {
                        joint0 = model->CreateJoint("joint0", "fixed", drone_0_link, CoM);
                        joint0enabled = true;
                        printf("Created fixed joint between drone0 and CoM\n");
                    }
                }
            }
        }
    private:
        physics::WorldPtr worldPtr;
        physics::ModelPtr model;
        physics::LinkPtr link;
        physics::LinkPtr red_attachment_point;
        physics::LinkPtr green_attachment_point;
        physics::LinkPtr blue_attachment_point;
        physics::LinkPtr CoM;
		physics::ModelPtr drone_0;
		physics::ModelPtr drone_1;
		physics::LinkPtr drone_0_link;
        physics::LinkPtr drone_1_link;
		physics::JointPtr joint0;
        physics::JointPtr joint1;
        bool joint0enabled = false;
        bool joint1enabled = false;
        zmq::context_t context;
        zmq::socket_t subscriber;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DSCC2020Init)
}