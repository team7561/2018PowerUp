package frc.team9761.robot.Subsystems

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.VictorSP
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.DoubleSolenoid
import frc.team9761.robot.Ports
import frc.team9761.robot.Speeds

class Wrist {
    val motor = VictorSP(Ports.WRIST_MOTOR_CHANNEL)
    val servo = Servo(Ports.WRIST_SERVO_CHANNEL)
    //val cylinder = DoubleSolenoid(0, 1)

    fun release() {
        setservoPosition(1.0)
    }

    fun hold() {
        setservoPosition(0.0)
    }

    fun raise() {
        setWristPower(Speeds.WRIST_RAISE_POWER)
    }

    fun lower() {
        setWristPower(Speeds.WRIST_LOWER_POWER)
    }

    fun stop() {
        setWristPower(0.0)
    }

    fun setservoPosition(position: Double) {
        SmartDashboard.putNumber("servo", position)
        motor.set(position)
    }

    fun setWristPower(wristPower: Double) {
        SmartDashboard.putNumber("wristPower", wristPower)
        motor.set(wristPower)
    }
}
