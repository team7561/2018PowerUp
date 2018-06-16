package frc.team6434.robot

import edu.wpi.first.wpilibj.GenericHID.Hand
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.XboxController
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.Spark

class Robot : IterativeRobot() {
    val XBOX_CONTROLLER_PORT = 0

    // CAN Bus Node IDs
    val LEFT_A_CANID = 1
    val LEFT_B_CANID = 2
    val RIGHT_A_CANID = 3
    val RIGHT_B_CANID = 4

    // Lift motor control channels
    val LIFT_A_CHANNEL = 1
    val LIFT_B_CHANNEL = 2
    val LIFT_C_CHANNEL = 3
    val LIFT_D_CHANNEL = 4

    lateinit var controller: XboxController

    lateinit var leftA: WPI_TalonSRX
    lateinit var leftB: WPI_TalonSRX
    lateinit var rightA: WPI_TalonSRX
    lateinit var rightB: WPI_TalonSRX

    lateinit var liftA: Spark
    lateinit var liftB: Spark
    lateinit var liftC: Spark
    lateinit var liftD: Spark

    override fun robotInit() {
        println("Hello Illawarra 9761")

        controller = XboxController(XBOX_CONTROLLER_PORT)

        leftA = WPI_TalonSRX(LEFT_A_CANID)
        leftB = WPI_TalonSRX(LEFT_B_CANID)
        rightA = WPI_TalonSRX(RIGHT_A_CANID)
        rightB = WPI_TalonSRX(RIGHT_B_CANID)

        liftA = Spark(LIFT_A_CHANNEL)
        liftB = Spark(LIFT_B_CHANNEL)
        liftC = Spark(LIFT_C_CHANNEL)
        liftD = Spark(LIFT_D_CHANNEL)


        leftB.follow(leftA)
        rightB.follow(rightA)
    }

    fun clamp(num: Double): Double {
      if (num < -1.0)
        return -1.0
      if (num > 1.0)
        return 1.0
      return num
    }

    fun optionallyReverse(num: Double, switch: Boolean): Double {
      if (switch)
        return -num
      
      return num
    }

    fun getLiftPower(leftBumper: Boolean, rightBumper: Boolean): Double {
      if (leftBumper)
        return -1.0
      if (rightBumper)
        return 1.0
      return 0.0
    }

    override fun teleopPeriodic() {
      val x = controller.getY(Hand.kLeft)
      val y = controller.getX(Hand.kRight)
      SmartDashboard.putNumber("x", x)
      SmartDashboard.putNumber("y", y)

      // based on getBButton(), reverse x
      val orx = optionallyReverse(x, controller.getBButton())
      SmartDashboard.putNumber("orx", orx)


      var leftPower = clamp(y + orx)
      var rightPower = clamp(y - orx)
      SmartDashboard.putNumber("leftPower", leftPower)
      SmartDashboard.putNumber("rightPower", rightPower)
      leftA.set(leftPower)
      rightA.set(rightPower)

      var liftPower = getLiftPower(controller.getBumper(Hand.kLeft), controller.getBumper(Hand.kRight))
      SmartDashboard.putNumber("liftPower", liftPower)
      liftA.set(liftPower)
      liftB.set(liftPower)
      liftC.set(-liftPower)
      liftD.set(-liftPower)
    }
}
