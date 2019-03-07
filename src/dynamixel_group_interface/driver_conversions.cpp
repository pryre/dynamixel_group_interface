#include <dynamixel_group_interface/interface_manager.h>

namespace DynamixelGroupInterface {

int InterfaceManager::convert_current_value( double current, int motor_number ) {
	return current / motor_value_to_current_;
}

double InterfaceManager::convert_value_current( int value, int motor_number ) {
	return (double)value * motor_value_to_current_;
}

int InterfaceManager::convert_velocity_value( double velocity,
	int motor_number ) {
	return velocity / motor_value_to_velocity_;
}

double InterfaceManager::convert_value_velocity( int value, int motor_number ) {
	return (double)value * motor_value_to_velocity_;
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
	double radian = (double)value * motor_value_to_position_;

	// Constrain angle from -M_PI to +M_PI
	radian = fmod( radian, 2 * M_PI );

	if ( radian < 0 )
		radian += 2 * M_PI;

	return radian - M_PI;
}

}
