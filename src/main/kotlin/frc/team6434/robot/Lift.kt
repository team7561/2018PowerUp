package frc.team6434.robot


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.Spark

class Lift {
      val liftA = Spark(Ports.LIFT_A_CHANNEL)
      val liftB = Spark(Ports.LIFT_B_CHANNEL)
      val liftC = Spark(Ports.LIFT_C_CHANNEL)
      val liftD = Spark(Ports.LIFT_D_CHANNEL)

      fun raise() {
        setLiftPower(Speeds.RAISE_POWER)
      }

      fun lower() {
        setLiftPower(Speeds.LOWER_POWER)
      }

      fun stop() {
        setLiftPower(0.0)
      }

      fun setLiftPower(liftPower: Double) {
        SmartDashboard.putNumber("liftPower", liftPower)
        liftA.set(liftPower)
        liftB.set(liftPower)
        liftC.set(-liftPower)
        liftD.set(-liftPower)

      }

      
}
