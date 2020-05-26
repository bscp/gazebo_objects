
// HEADER INCLUDE
#include <gazebo_object/GazeboPlugin.hpp>

// SYSTEM INCLUDES
#include <string>

#define OBJECTS_TOPIC_NAME "objects"
#define OBJECTS_TOPIC_BUFFER_SIZE 25
#define OBJECT_NAME_PARAM_NAME "object_name"


namespace gazebo_object {

    GazeboPlugin::GazeboPlugin()
    {
    }


    void GazeboPlugin::Load(ModelPtr model_ptr, sdf::ElementPtr /*sdf_ptr*/)
    {
        this->model_ptr = model_ptr;
        register_update_callback();
        register_reset_callback();
        register_object_state_publisher();
        this->object_state_msg.name = model_ptr->GetName();
    }


    void GazeboPlugin::register_update_callback()
    {
        this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboPlugin::on_update, this));
    }


    void GazeboPlugin::register_reset_callback()
    {
        this->reset_connection = gazebo::event::Events::ConnectWorldReset(
            std::bind(&GazeboPlugin::on_reset, this));
    }


    void GazeboPlugin::register_object_state_publisher()
    {
        node_handle_ptr = boost::make_shared<ros::NodeHandle>("~");
        object_state_publisher = node_handle_ptr->advertise<ObjectStateMsg>(
            OBJECTS_TOPIC_NAME, OBJECTS_TOPIC_BUFFER_SIZE);
    }


    void GazeboPlugin::publish_message()
    {
        object_state_publisher.publish(object_state_msg);
    }
    

    void GazeboPlugin::on_update()
    {
        // TODO : set new position
        // TODO : set new orientation
        // TODO : set new acceleration
        // TODO : throttle publish rate
        publish_message();
    }
    

    void GazeboPlugin::on_reset()
    {
        this->model_ptr->Fini();
    }

}
