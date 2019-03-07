#include <dynamixel_group_interface/interface_manager.h>

namespace DynamixelGroupInterface {

//Current unit: 2.69 mA
//Velocity unit: 0.229 RPM
//Position unit: 0.088 Deg

//Hardcoded for XM430-W350
//const double velocity_to_value_ratio = 1.0 / ( 0.229 * 2.0 * M_PI / 60.0 );
//const double position_to_value_ratio = 1.0 / ( 0.088 * M_PI / 180.0 );

/*
int InterfaceManager::convert_effort_value( double effort, int motor_number ) {
	return (int)( torque * dxl_[motor_number].getTorqueToCurrentValueRatio() );
}

double InterfaceManager::convert_value_effort( int value, int motor_number ) {
	return (double)value / dxl_[motor_number].getTorqueToCurrentValueRatio();
}
*/

int InterfaceManager::convert_velocity_value( double velocity,
	int motor_number ) {
	return velocity / value_to_velocity_ratio_;
}

double InterfaceManager::convert_value_velocity( int value, int motor_number ) {
	return (double)value * value_to_velocity_ratio_;
}

int InterfaceManager::convert_radian_value( double radian, int motor_number ) {
	int64_t value = 0;
	int64_t rad_max = dxl_[motor_number].getMaxRadian();
	int64_t rad_min = dxl_[motor_number].getMinRadian();
	int64_t val_max = dxl_[motor_number].getValueOfMaxRadianPosition();
	int64_t val_min = dxl_[motor_number].getValueOfMinRadianPosition();
	int64_t val_zero = dxl_[motor_number].getValueOfZeroRadianPosition();

	if ( radian > 0.0 ) {
		value = ( radian * ( val_max - val_zero ) / rad_max ) + val_zero;
	} else if ( radian < 0.0 ) {
		value = ( radian * ( val_min - val_zero ) / rad_min ) + val_zero;
	} else {
		value = val_zero;
	}

	value = ( value > val_max ) ? val_max : ( value < val_min ) ? val_min : value;

	return value;
}

double InterfaceManager::convert_value_radian( int value, int motor_number ) {
	double radian = (double)value * value_to_position_ratio_;

	// Constrain angle from -M_PI to +M_PI
	radian = fmod( radian, 2 * M_PI );

	if ( radian < 0 )
		radian += 2 * M_PI;

	return radian - M_PI;
}

}
