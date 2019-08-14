#include <ros/ros.h>

#include <dynamixel_group_interface/interface_manager.h>
#include <dynamixel_group_interface/EnableTorque.h>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_workbench_toolbox/dynamixel_tool.h>

#include <sensor_msgs/JointState.h>

#include <math.h>
#include <string>
#include <vector>

namespace DynamixelGroupInterface {

InterfaceManager::InterfaceManager()
	: nh_()
	, nhp_( "~" )
	,
	// motor_output_mode_(MOTOR_MODE_INVALID),
	param_update_rate_( 50.0 )
	, param_port_name_( "/dev/ttyUSB0" )
	, param_port_buad_( 57600 )
	, param_port_version_( 0.0 )
	, motor_model_("")
	, motor_value_to_position_(0.0)
	, motor_value_to_velocity_(0.0)
	, motor_value_to_current_(0.0)
	, motor_indirect_addr_(0)
	, motor_indirect_len_(0)
	, motor_current_torque_m_(0.0)
	, motor_current_torque_c_(0.0)
	, motor_current_torque_cutoff_(0.0)
	, param_frame_id_( "robot" )
	, motor_ref_last_( InterfaceJoint::CurrentReference::Unset ) {

	// ROS setup
	nhp_.param( "port_name", param_port_name_, param_port_name_ );
	nhp_.param( "port_baud", param_port_buad_, param_port_buad_ );
	nhp_.param( "protocol", param_port_version_, param_port_version_ );
	nhp_.param( "frame_id", param_frame_id_, param_frame_id_ );
	nhp_.param( "update_rate", param_update_rate_, param_update_rate_ );

	// Motor model setup
	nhp_.param( "group/model", motor_model_, motor_model_ );

	ROS_ASSERT_MSG( nhp_.hasParam("models/" + motor_model_ + "/name") &&
					nhp_.hasParam("models/" + motor_model_ + "/value_to_position") &&
					nhp_.hasParam("models/" + motor_model_ + "/value_to_velocity") &&
					nhp_.hasParam("models/" + motor_model_ + "/value_to_current") &&
					nhp_.hasParam("models/" + motor_model_ + "/indirect_address_1/addr") &&
					nhp_.hasParam("models/" + motor_model_ + "/indirect_address_1/len") &&
					nhp_.hasParam("models/" + motor_model_ + "/current_torque_curve/m") &&
					nhp_.hasParam("models/" + motor_model_ + "/current_torque_curve/c") &&
					nhp_.hasParam("models/" + motor_model_ + "/current_torque_curve/cutoff"),
					"Could not find all model parameters for '%s'", motor_model_.c_str());

	nhp_.getParam("models/" + motor_model_ + "/name", motor_model_name_);
	nhp_.getParam("models/" + motor_model_ + "/value_to_position", motor_value_to_position_);
	nhp_.getParam("models/" + motor_model_ + "/value_to_velocity", motor_value_to_velocity_);
	nhp_.getParam("models/" + motor_model_ + "/value_to_current", motor_value_to_current_);
	nhp_.getParam("models/" + motor_model_ + "/indirect_address_1/addr", motor_indirect_addr_);
	nhp_.getParam("models/" + motor_model_ + "/indirect_address_1/len", motor_indirect_len_);
	nhp_.getParam("models/" + motor_model_ + "/current_torque_curve/m", motor_current_torque_m_);
	nhp_.getParam("models/" + motor_model_ + "/current_torque_curve/c", motor_current_torque_c_);
	nhp_.getParam("models/" + motor_model_ + "/current_torque_curve/cutoff", motor_current_torque_cutoff_);

	bool control_items_ok = true;
	{
		// XXX: This is a pretty bad hack for this, but is unavoidable.
		//		We have to get the control table somehow, and all motors
		//		must be the same, so just load in the control items
		//		for motor #1. Really we would want a 2D motor, but that just
		//		doesn't work if we want to use GroupSync
		DynamixelTool dummy_tool;
		dummy_tool.addTool( motor_model_name_.c_str(), (uint8_t)0 );
		for ( int j = 0; j < DCI_NUM_ITEMS; j++ ) {
			const ControlItem* ci = dummy_tool.getControlItem( DYNAMIXEL_CONTROL_ITEMS_STRING[j].c_str() );
			if(ci != NULL) {
				dynamixel_control_items_.push_back(*ci);

				ROS_DEBUG( "Loaded ControlTableItem \"%s\": %i; %i",
					DYNAMIXEL_CONTROL_ITEMS_STRING[j].c_str(),
					dynamixel_control_items_[j].address,
					dynamixel_control_items_[j].data_length );
			} else  {
				ROS_FATAL("Unable to load control item: %s", DYNAMIXEL_CONTROL_ITEMS_STRING[j].c_str());
				control_items_ok = false;
				break;
			}
		}
	}

	// sub_setpoints_ = nh_.subscribe<sensor_msgs::JointState>( "joint_setpoints",
	// 10, &InterfaceManager::callback_setpoints, this );

	// Dynamixel Setup
	portHandler_ = dynamixel::PortHandler::getPortHandler( param_port_name_.c_str() );
	packetHandler_ = dynamixel::PacketHandler::getPacketHandler( param_port_version_ );

	bool port_ok = false;

	/*
	ROS_WARN("START TOOL TEST");
	DynamixelTool tool;
	tool.addTool("XM430-W350", 3);
	ROS_INFO("id: %d, mdl: %d, cnt: %d", tool.dxl_info_[tool.dxl_info_cnt_-1].id,
	tool.dxl_info_[tool.dxl_info_cnt_-1].model_num, tool.dxl_info_cnt_);
	ControlTableItem* item = tool.getControlItem("Torque_Enable");
	ROS_INFO("addr: %d, len: %d", item->address, item->data_length);
	ROS_WARN("END TOOL TEST");

	shutdown_node();
	*/

	// Open port
	if(control_items_ok) {
		if ( portHandler_->openPort() ) {
			ROS_INFO( "Succeeded to open the port(%s)!", param_port_name_.c_str() );

			// Set port baudrate
			if ( portHandler_->setBaudRate( param_port_buad_ ) ) {
				ROS_INFO( "Succeeded to change the baudrate(%d)!",
					portHandler_->getBaudRate() );
				port_ok = true;
			} else {
				ROS_ERROR( "Failed to change the baudrate!" );
			}
		} else {
			ROS_ERROR( "Failed to open the port!" );
		}
	}

	if ( control_items_ok && port_ok ) {
		ROS_INFO( "Scanning for motors..." );

		if ( add_motors() ) {
			ROS_INFO( "All motors successfully added!" );

			ROS_INFO( "Initializing SyncRead..." );
			initSyncRead();
			ROS_INFO( "SyncRead done!" );

			pub_states_ = nh_.advertise<sensor_msgs::JointState>( "joint_states", 10 );

			timer_ = nhp_.createTimer( ros::Duration( 1.0 / param_update_rate_ ),
				&InterfaceManager::callback_timer, this );
			srv_enable_torque_specific_ = nhp_.advertiseService(
				"enable_torque_specific", &InterfaceManager::enable_torque_specific,
				this );
			srv_enable_torque_all_ = nhp_.advertiseService(
				"enable_torque_all", &InterfaceManager::enable_torque_all, this );

			ROS_INFO( "Dynamixel interface started successfully!" );
		} else {
			ROS_ERROR( "Failed to contact motors!" );
			shutdown_node();
		}
	} else {
		shutdown_node();
	}
}

InterfaceManager::~InterfaceManager() {
	shutdown_node();
}

uint8_t InterfaceManager::get_id( int motor_number ) {
	return *(dxl_[motor_number].getID()); //dxl_[motor_number].dxl_info_[dxl_[motor_number].dxl_info_cnt_ - 1].id;
}

void InterfaceManager::clean_joint_interfaces( void ) {
	// Delete the created joint interfaces
	for ( int i = 0; i < joint_interfaces_.size(); i++ ) {
		delete joint_interfaces_[i];
	}

	//Clear the vector entries
	joint_interfaces_.clear();
}

void InterfaceManager::shutdown_node( void ) {
	ROS_ERROR( "Shutting down dynamixel interface" );

	// Safe shutdown of motors
	for ( int i = 0; i < dxl_.size(); i++ )
		set_torque_enable( i, false );

	portHandler_->closePort();

	clean_joint_interfaces();

	ros::shutdown();
}

bool InterfaceManager::set_torque_enable( int motor_number, bool onoff ) {
	return writeMotorState( DCI_TORQUE_ENABLE, motor_number, onoff );
}

bool InterfaceManager::enable_torque_specific(
	dynamixel_group_interface::EnableTorque::Request& req,
	dynamixel_group_interface::EnableTorque::Response& res ) {
	bool success = true;

	if ( req.set_enable.size() == dxl_.size() ) {
		for ( int i = 0; i < dxl_.size(); i++ ) {
			if ( req.set_enable[i] ) {
				ROS_INFO( "Turning on motor_%i!", i );
			} else {
				ROS_INFO( "Turning off motor_%i!", i );
			}

			res.success &= set_torque_enable( i, req.set_enable[i] );
		}
	} else {
		ROS_ERROR( "Torque enable vector must be same size as motors!" );
		success = false;
	}

	res.success = success;

	return true;
}

bool InterfaceManager::enable_torque_all( std_srvs::SetBool::Request& req,
	std_srvs::SetBool::Response& res ) {
	res.success = true;

	for ( int i = 0; i < dxl_.size(); i++ ) {
		if ( req.data ) {
			ROS_INFO( "Turning on motor_%i!", i );
		} else {
			ROS_INFO( "Turning off motor_%i!", i );
		}

		res.success &= set_torque_enable( i, req.data );
	}

	if ( req.data ) {
		res.message = "Turning on all motors";
	} else {
		res.message = "Turning off all motors";
	}

	return true;
}

void InterfaceManager::init_motor( uint8_t motor_id,
								   double protocol_version,
								   std::string motor_name ) {
	DynamixelTool tool;
	dxl_.push_back( tool );
	dxl_.back().addTool( motor_model_.c_str(), motor_id );
	dxl_names_.push_back( motor_name );
}

bool InterfaceManager::add_motors() {
	//Will fail the motor setup if something goes wrong at any point
	bool success = true;

	// Make sure we start fresh
	clean_joint_interfaces();

	//Start looping through motors definitions
	int i = 0;
	bool got_control_items = false;
	while ( nhp_.hasParam( "group/motor_" + std::to_string( i ) + "/id" ) &&
			nhp_.hasParam( "group/motor_" + std::to_string( i ) + "/name" ) &&
			nhp_.hasParam( "group/motor_" + std::to_string( i ) + "/ref_timeout" ) &&
			nhp_.hasParam( "group/motor_" + std::to_string( i ) + "/profile/acceleration" ) &&
			nhp_.hasParam( "group/motor_" + std::to_string( i ) + "/profile/velocity" ) ) {

		std::string motor_name;
		std::string motor_model;
		int motor_id;
		double protocol_version;
		nhp_.getParam( "group/motor_" + std::to_string( i ) + "/name", motor_name );
		nhp_.getParam( "group/motor_" + std::to_string( i ) + "/id", motor_id );
		protocol_version = param_port_version_;

		ROS_INFO( "Initializing motor #%i", i );
		init_motor( (uint8_t)motor_id, protocol_version, motor_name );

		ROS_INFO( "Contacting motor #%i", i );
		int64_t tmp_val;

		if ( readMotorState( DCI_TORQUE_ENABLE, i, &tmp_val ) ) {
			set_torque_enable( i, false ); // Make sure motor is not turned on
			ROS_INFO( "Motor %i successfully added", i );

			// Add in a new joint interface
			double ref_timeout;
			nhp_.getParam( "group/motor_" + std::to_string( i ) + "/ref_timeout", ref_timeout );
			joint_interfaces_.push_back( new InterfaceJoint( nh_, motor_name, ref_timeout ) );

			//Set movement profile data
			double profile_accel;
			double profile_vel;
			nhp_.getParam( "group/motor_" + std::to_string( i ) + "/profile/acceleration", profile_accel );
			nhp_.getParam( "group/motor_" + std::to_string( i ) + "/profile/velocity", profile_vel );
			writeMotorState( DCI_PROFILE_ACCELERATION, i, profile_accel );
			writeMotorState( DCI_PROFILE_VELOCITY, i, profile_vel );
			ROS_INFO( "Motor %i successfully movement profile set", i );
		} else {
			ROS_ERROR( "Motor %i could not be read!", i );
			success = false;
		}

		i++;
	}

	// Fail if no motor listing was found in the parameters
	if( i == 0 && got_control_items ) {
		ROS_ERROR( "No motor definitions found in group!" );
		success = false;
	}

	ROS_ASSERT_MSG( dxl_.size() == joint_interfaces_.size(),
		"Motor and interface number does not match (%i!=%i)",
		(int)dxl_.size(), (int)joint_interfaces_.size() );

	ROS_ASSERT_MSG( dxl_.size() == dxl_names_.size(),
		"Motor and name number does not match (%i!=%i)",
		(int)dxl_.size(), (int)dxl_names_.size() );

	return success;
}

void InterfaceManager::callback_timer( const ros::TimerEvent& e ) {
	sensor_msgs::JointState joint_states;

	joint_states.header.stamp = e.current_real;
	joint_states.header.frame_id = param_frame_id_;

	joint_states.position.clear();
	joint_states.velocity.clear();
	joint_states.effort.clear();

	// Allocated as MxN vector of motor states
	//	M is motor_id's
	//	N is [torque_enable, position, velocity, current]
	std::vector<std::vector<std::int32_t>> states;

	if ( doSyncRead( &states ) ) {
		joint_states.name = dxl_names_;

		for ( int i = 0; i < dxl_.size(); i++ ) {
			joint_states.position.push_back( convert_value_radian( states[i][1], i ) );
			joint_states.velocity.push_back( convert_value_velocity( states[i][2], i ) );
			joint_states.effort.push_back(  estimate_torque_from_current (
											convert_value_current( states[i][3], i ) ) );

			/*
      if(states[i][0] && (motor_output_mode != MOTOR_MODE_INVALID)) {
              switch(motor_output_mode) {
                      case MOTOR_MODE_TORQUE: {
                              writeMotorState("goal_current", i,
      convert_torque_value(joint_setpoints_.effort[i], i));

                              break;
                      }
                      case MOTOR_MODE_VELOCITY: {
                              writeMotorState("goal_velocity", i,
      convert_velocity_value(joint_setpoints_.velocity[i], i));

                              break;
                      }
                      case MOTOR_MODE_POSITION: {
                              writeMotorState("goal_position", i,
      convert_radian_value(joint_setpoints_.position[i], i));

                              break;
                      }
                      default: {
                              ROS_ERROR("Setpoint has not been received yet, but
      motors are on!");
                      }
              }
      }
      */
		}

		pub_states_.publish( joint_states );
	}

	InterfaceJoint::CurrentReference motor_ref_type = InterfaceJoint::CurrentReference::Unset;
	bool motor_ref_consistent = true;
	std::vector<double> motor_refs;

	for ( int i = 0; i < joint_interfaces_.size(); i++ ) {
		if ( i == 0 )
			motor_ref_type = joint_interfaces_[i]->get_reference_type();

		if ( ( motor_ref_type == InterfaceJoint::CurrentReference::Unset ) || ( motor_ref_type != joint_interfaces_[i]->get_reference_type() ) ) {

			motor_ref_consistent = false;
			break;
		}
	}

	if ( motor_ref_consistent ) {
		motor_refs.resize( dxl_.size() );

		for ( int i = 0; i < dxl_.size(); i++ ) {
			if ( !joint_interfaces_[i]->get_reference( motor_refs[i] ) ) {
				motor_ref_consistent = false;
				break;
			}
		}
	}

	if ( motor_ref_consistent ) {
		bool motor_ref_changed = false;

		if ( motor_ref_last_ != motor_ref_type ) {
			motor_ref_last_ = motor_ref_type;
			motor_ref_changed = true;
		}

		// Use the lowest level mode that has been sent
		if ( motor_ref_type == InterfaceJoint::CurrentReference::Effort ) {
			if ( motor_ref_changed ) {
				/*
				ROS_INFO( "Effort control setpoint accepted" );
				for ( int i = 0; i < dxl_.size(); i++ ) {
					set_torque_enable( i, false );
					writeMotorState( DCI_OPERATING_MODE, i, MOTOR_MODE_TORQUE );
				}
				*/

				ROS_WARN( "--- EFFORT INTERFACE UNAVAILABLE ---" );
			}

			//doSyncWrite( DCI_GOAL_CURRENT, &motor_refs );
		} else if ( motor_ref_type == InterfaceJoint::CurrentReference::Velocity ) {
			if ( motor_ref_changed ) {
				ROS_INFO( "Velocity control setpoint accepted" );
				for ( int i = 0; i < dxl_.size(); i++ ) {
					set_torque_enable( i, false );
					writeMotorState( DCI_OPERATING_MODE, i, MOTOR_MODE_VELOCITY );

					//ROS_WARN( "--- HACK TO SET MOVEMENT PROFILE ---" );
					//writeMotorState( DCI_PROFILE_ACCELERATION, i, 50 );
				}
			}

			doSyncWrite( DCI_GOAL_VELOCITY, &motor_refs );
		} else if ( motor_ref_type == InterfaceJoint::CurrentReference::Position ) {
			if ( motor_ref_changed ) {
				ROS_INFO( "Position control setpoint accepted" );
				for ( int i = 0; i < dxl_.size(); i++ ) {
					set_torque_enable( i, false );
					writeMotorState( DCI_OPERATING_MODE, i, MOTOR_MODE_POSITION );

					//ROS_WARN( "--- HACK TO SET MOVEMENT PROFILE ---" );
					//writeMotorState( DCI_PROFILE_ACCELERATION, i, 50 );
					//writeMotorState( DCI_PROFILE_VELOCITY, i, 50 );
				}
			}

			doSyncWrite( DCI_GOAL_POSITION, &motor_refs );
		}
	} else {
		ROS_WARN_THROTTLE(
			10.0, "Waiting for motor command references to be set and consistent" );
	}
}

}

/*
void InterfaceManager::callback_setpoints(const
sensor_msgs::JointState::ConstPtr& msg_in) {
        //TODO: expect a stream?

        bool success = true;
        int num_motors = dxl_.size();

        //Checks to make sure that at least one input is of the right size
        bool input_size_name_ok = msg_in->name.size() == num_motors;
        bool input_size_position_ok = msg_in->position.size() == num_motors;
        bool input_size_velocity_ok = msg_in->velocity.size() == num_motors;
        bool input_size_effort_ok = msg_in->effort.size() == num_motors;

        bool input_ok = input_size_name_ok && (input_size_position_ok ||
input_size_velocity_ok || input_size_effort_ok);

        std::string failure_reason;

        if( input_ok ) {
                joint_setpoints_ = *msg_in;

                //Use the lowest level mode that has been sent
                if(input_size_effort_ok) {
                        if(motor_output_mode_ != MOTOR_MODE_TORQUE) {
                                ROS_INFO("Torque control setpoint accepted");
                                for(int i=0; i<num_motors; i++) {
                                        set_torque_enable(i, false);
                                        writeMotorState(DCI_OPERATING_MODE, i,
MOTOR_MODE_TORQUE);
                                        motor_output_mode_ = MOTOR_MODE_TORQUE;
                                }
                        }
                } else if( input_size_velocity_ok ) {
                        if(motor_output_mode_ != MOTOR_MODE_VELOCITY) {
                                ROS_INFO("Velocity control setpoint accepted");
                                for(int i=0; i<num_motors; i++) {
                                        set_torque_enable(i, false);
                                        writeMotorState(DCI_OPERATING_MODE, i,
MOTOR_MODE_VELOCITY);
                                        motor_output_mode_ =
MOTOR_MODE_VELOCITY;
                                }
                        }
                } else if( input_size_position_ok ) {
                        if(motor_output_mode_ != MOTOR_MODE_POSITION) {
                                ROS_INFO("Position control setpoint accepted");
                                for(int i=0; i<num_motors; i++) {
                                        set_torque_enable(i, false);
                                        writeMotorState(DCI_OPERATING_MODE, i,
MOTOR_MODE_POSITION);
                                        motor_output_mode_ =
MOTOR_MODE_POSITION;

                                        ROS_WARN("--- HACK TO SET MOVEMENT
PROFILE ---");
                                        writeMotorState(DCI_PROFILE_ACCELERATION,
i, 50);
                                        writeMotorState(DCI_PROFILE_VELOCITY, i,
50);
                                }
                        }
                } else {
                        failure_reason = "Something went wrong when selecting
setpoint mode!";
                        success = false;
                }
        } else {
                if(!input_size_name_ok) {
                        //TODO: We should probably actual match motor inputs
with the specified names
                        failure_reason = "Name vector length doesn't match
motors avaliable";
                } else if ( (msg_in->effort.size() > 0) &&
!input_size_effort_ok) {
                        failure_reason = "Effort vector length doesn't match
motors avaliable";
                } else if ( (msg_in->velocity.size() > 0) &&
!input_size_velocity_ok) {
                        failure_reason = "Velocity vector length doesn't match
motors avaliable";
                } else if ( (msg_in->position.size() > 0) &&
!input_size_position_ok) {
                        failure_reason = "Position vector length doesn't match
motors avaliable";
                } else {
                        failure_reason = "Unkown sepoint error!";
                }

                success = false;
        }

        if(!success) {
                ROS_WARN_THROTTLE(1.0, "Ignoring setpoint: %s",
failure_reason.c_str());
        }
}
*/
