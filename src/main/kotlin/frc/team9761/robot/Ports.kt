package frc.team9761.robot

object Ports {
    const val XBOX_CONTROLLER_PORT = 0

    // CAN Bus Node IDs
    const val LIFT_LEFT_A_CANID = 1
    const val LIFT_LEFT_B_CANID = 2
    const val LIFT_RIGHT_A_CANID = 3
    const val LIFT_RIGHT_B_CANID = 4

    // PWM control channels
    const val DRIVE_RIGHT_A_CHANNEL = 1
    const val DRIVE_RIGHT_B_CHANNEL = 2
    const val DRIVE_LEFT_A_CHANNEL = 3
    const val DRIVE_LEFT_B_CHANNEL = 4
    const val WRIST_MOTOR_CHANNEL = 5
    const val INTAKE_LEFT_CHANNEL = 6
    const val INTAKE_RIGHT_CHANNEL = 7
    const val WRIST_SERVO_CHANNEL = 8

    // DIO ports
    const val ENCODER_LEFT_A_CHANNEL = 0
    const val ENCODER_LEFT_B_CHANNEL = 1
    const val ENCODER_RIGHT_A_CHANNEL = 2
    const val ENCODER_RIGHT_B_CHANNEL = 3

    const val ARDUINO_SPEED_CHANNEL = 6
    const val ARDUINO_POSITION_BIT_ONE_CHANNEL = 7
    const val ARDUINO_POSITION_BIT_TWO_CHANNEL = 8
    const val ARDUINO_INTAKE_CHANNEL = 9

    // Analog Channels
    const val POSITION_CHANNEL = 1
    const val ULTRASONIC_CHANNEL = 2
}
