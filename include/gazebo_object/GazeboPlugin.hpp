
#ifndef gazebo_object_OBJECT_PLUGIN
#define gazebo_object_OBJECT_PLUGIN

// SYSTEM INCLUDES
#include <memory>

// 3TH PARTY INCLUDES
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// PROJECT INCLUDES
#include <gazebo_object/ObjectStateMsg.h>


namespace gazebo_object
{
    typedef gazebo::physics::ModelPtr ModelPtr;
    typedef gazebo::event::ConnectionPtr ConnectionPtr;

    class GazeboPlugin : public gazebo::ModelPlugin
    {
    public:
        GazeboPlugin();

        virtual ~GazeboPlugin() final = default;

        /// executed when a new instance of the plugin is created
        void Load(
            ModelPtr parent_model_ptr,
            sdf::ElementPtr sdf_ptr)
            override;

    private:

        /// connect on_update method to the Gazebo update event
        void register_update_callback();
        
        /// connect on_reset method to the Gazebo reset event
        void register_reset_callback();

        /// registers the object state publisher at roscore 
        void register_object_state_publisher();

        /// executed when the simulation gets updated
        void on_update();

        /// executed when the simulation world performs a reset
        void on_reset();

        /// publishes the object_state_msg to publisher
        void publish_message();
        
        ModelPtr model_ptr;
        ConnectionPtr update_connection;
        ConnectionPtr reset_connection;
        ros::NodeHandlePtr node_handle_ptr;
        ros::Publisher object_state_publisher;
        ObjectStateMsg object_state_msg;
    };

    GZ_REGISTER_MODEL_PLUGIN(GazeboPlugin)
}

#endif
