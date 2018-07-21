package frc.team9761.robot


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.VictorSP
import edu.wpi.first.wpilibj.Servo

class Wrist {
      val motor = VictorSP(Ports.WRIST_MOTOR_CHANNEL)
      val servo = Servo(Ports.WRIST_SERVO_CHANNEL)

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
