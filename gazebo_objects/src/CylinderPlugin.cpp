
#include <gazebo_objects/CylinderPlugin.hpp>
#include <iostream>

namespace gazebo {

    CylinderPlugin::~CylinderPlugin()
    {
        // temp alternative fix for event::Events::ConnectDeleteEntity
        OnDelete();
    }

    void CylinderPlugin::Load(physics::ModelPtr parentPtr, sdf::ElementPtr sdfPtr)
    {
        nodeHandlePtr = std::make_unique<ros::NodeHandle>("~");

        publisher = nodeHandlePtr->advertise<moveit_msgs::CollisionObject>("collision_object", 1);
        modelPtr = parentPtr;

        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&CylinderPlugin::OnUpdate, this));

        //// does not work: bug report: https://bitbucket.org/osrf/gazebo/issues/2171/delete-entity-event-is-never-called
        //deleteConnection = event::Events::ConnectDeleteEntity(
        //std::bind(&CylinderPlugin::OnDelete, this));


        isDetected = false;
        if (sdfPtr->HasElement("detected"))
        {
            isDetected = sdfPtr->Get<bool>("detected");
        }

    }


    void CylinderPlugin::OnUpdate()
    {
        collisionObject.operation = moveit_msgs::CollisionObject::MOVE;
        publishMessage();
    }


    void CylinderPlugin::OnDelete()
    {
        collisionObject.operation = moveit_msgs::CollisionObject::REMOVE;
        publishMessage();
    }


    void CylinderPlugin::publishMessage()
    {
        if (isDetected)
        {
            updateMessage();

            if (collisionObject.id.empty())
            {
                initializeMessage();
            }

            planning_scene_interface.applyCollisionObject(collisionObject);
        }
    }


    void CylinderPlugin::initializeMessage()
    {
        collisionObject.operation = moveit_msgs::CollisionObject::ADD;

        collisionObject.header.frame_id = "world";
        collisionObject.id = modelPtr->GetName();

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        
        primitive.dimensions.resize(2);
        auto shapePtr = modelPtr->GetLinks()[0]->GetCollisions()[0]->GetShape();
        auto *cylinderShapePtr = dynamic_cast<gazebo::physics::CylinderShape*>(shapePtr.get());

        primitive.dimensions[0] = cylinderShapePtr->GetLength();
        primitive.dimensions[1] = cylinderShapePtr->GetRadius();

        collisionObject.primitives.clear();
        collisionObject.primitives.push_back(primitive);
    }


    void CylinderPlugin::updateMessage()
    {
        const auto &gazeboPose = modelPtr->WorldPose();
        const auto &gazeboPosition = gazeboPose.Pos();
        const auto &gazeboRotation = gazeboPose.Rot();

        geometry_msgs::Pose pose;
        pose.position.x = gazeboPosition.X();
        pose.position.y = gazeboPosition.Y();
        pose.position.z = gazeboPosition.Z();

        pose.orientation.x = gazeboRotation.X();
        pose.orientation.y = gazeboRotation.Y();
        pose.orientation.z = gazeboRotation.Z();
        pose.orientation.w = gazeboRotation.W();

        collisionObject.primitives.clear();
        collisionObject.primitive_poses.clear();
        collisionObject.primitive_poses.push_back(pose);
    }

}
