package frc.team6434.robot


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.Spark

class Wrist {
      val motor = Spark(Ports.WRIST_CHANNEL)
      
      fun raise() {
        setWristPower(Speeds.RAISE_POWER)
      }

      fun lower() {
        setWristPower(Speeds.LOWER_POWER)
      }

      fun stop() {
        setWristPower(0.0)
      }

      fun setWristPower(wristPower: Double) {
        SmartDashboard.putNumber("wristPower", wristPower)
        motor.set(wristPower)
        
      }

      
}
