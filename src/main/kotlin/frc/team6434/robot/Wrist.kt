package frc.team6434.robot


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.VictorSP
class Wrist {
      val motor = VictorSP(Ports.WRIST_CHANNEL)
      
      fun raise() {
        setWristPower(Speeds.WRIST_RAISE_POWER)
      }

      fun lower() {
        setWristPower(Speeds.WRIST_LOWER_POWER)
      }

      fun stop() {
        setWristPower(0.0)
      }

      fun setWristPower(wristPower: Double) {
        SmartDashboard.putNumber("wristPower", wristPower)
        motor.set(wristPower)
        
      }

      
}
