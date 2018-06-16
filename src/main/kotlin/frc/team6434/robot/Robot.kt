package frc.team6434.robot

import edu.wpi.first.wpilibj.GenericHID.Hand
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.XboxController
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class Robot : IterativeRobot() {
 
    lateinit var controller: XboxController

    lateinit var drivetrain: Drivetrain
    lateinit var lift: Lift
  
    override fun robotInit() {
        println("Hello Illawarra 9761")

        controller = XboxController(XBOX_CONTROLLER_PORT)

        drivetrain = Drivetrain()
        lift = Lift()
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
           
          }


    var startTime: Long = 0

    override fun autonomousInit() {
      startTime = System.currentTimeMillis() 

    }

    override fun autonomousPeriodic() {
      val currentTime = System.currentTimeMillis() 
      val elapsedTime = currentTime-startTime


      var drivePower = if (elapsedTime < CROSS_LINE_DURATION) CROSS_LINE_POWER else 0.0
      drivetrain.setPower(drivePower, drivePower) 
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
      drivetrain.setPower(leftPower, rightPower)


      if (leftBumper)
        lift.raise()
      else if (rightBumper)
        lift.lower()
      else
        lift.stop()     
    }
}
