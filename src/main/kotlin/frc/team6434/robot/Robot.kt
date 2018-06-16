package frc.team6434.robot

import edu.wpi.first.wpilibj.GenericHID.Hand
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.XboxController
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class Robot : IterativeRobot() {
    val XBOX_CONTROLLER_PORT = 0

    // CAN Bus Node IDs
    val LEFT_A_CANID = 1
    val LEFT_B_CANID = 2
    val RIGHT_A_CANID = 3
    val RIGHT_B_CANID = 4

    lateinit var controller: XboxController

    lateinit var leftA: WPI_TalonSRX
    lateinit var leftB: WPI_TalonSRX
    lateinit var rightA: WPI_TalonSRX
    lateinit var rightB: WPI_TalonSRX

    override fun robotInit() {
        println("Hello Illawarra 9761")

        controller = XboxController(XBOX_CONTROLLER_PORT)

        leftA = WPI_TalonSRX(LEFT_A_CANID)
        leftB = WPI_TalonSRX(LEFT_B_CANID)
        rightA = WPI_TalonSRX(RIGHT_A_CANID)
        rightB = WPI_TalonSRX(RIGHT_B_CANID)

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

    override fun teleopPeriodic() {
      val x = controller.getX(Hand.kLeft)
      val y = controller.getY(Hand.kRight)
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
    }
}
