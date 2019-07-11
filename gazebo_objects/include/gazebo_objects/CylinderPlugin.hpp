
#ifndef GAZEBO_OBJECTS_CYLINDER_PLUGIN
#define GAZEBO_OBJECTS_CYLINDER_PLUGIN

#include <memory>
#include <string>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace gazebo {

    class CylinderPlugin : public ModelPlugin
    {
    public:
        virtual ~CylinderPlugin() final;

        void Load(physics::ModelPtr parentPtr, sdf::ElementPtr sdfPtr) override;

        void OnUpdate();

        void OnDelete();

    protected:
        void initializeMessage();

        void updateMessage();

        void publishMessage();

    private:
        physics::ModelPtr modelPtr;
        event::ConnectionPtr updateConnection;

        ros::NodeHandlePtr nodeHandlePtr;
        ros::Publisher publisher;

        moveit_msgs::CollisionObject collisionObject;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        bool isDetected;
    };

    GZ_REGISTER_MODEL_PLUGIN(CylinderPlugin)
}

#endif
