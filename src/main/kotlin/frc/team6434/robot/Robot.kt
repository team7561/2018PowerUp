package frc.team6434.robot

import edu.wpi.first.wpilibj.GenericHID.Hand
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class Robot : IterativeRobot() {
 
    lateinit var controller: XboxController

    lateinit var drivetrain: Drivetrain
    lateinit var lift: Lift
    lateinit var wrist: Wrist
    lateinit var intake: Intake
    
    override fun robotInit() {
        println("Hello Illawarra 9761")

        controller = XboxController(Ports.XBOX_CONTROLLER_PORT)

        drivetrain = Drivetrain()
        lift = Lift()
        wrist = Wrist()
        intake = Intake()
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


    var startTime: Long = 0

    override fun autonomousInit() {
      startTime = System.currentTimeMillis() 

    }

    override fun autonomousPeriodic() {
      val currentTime = System.currentTimeMillis() 
      val elapsedTime = currentTime-startTime


      var drivePower = if (elapsedTime < Speeds.CROSS_LINE_DURATION) Speeds.CROSS_LINE_POWER else 0.0
      drivetrain.setPower(drivePower, drivePower) 


      if (elapsedTime < Speeds.WRIST_LOWER_DURATION)
        wrist.lower()
      else 
        wrist.stop()
    }



    override fun teleopPeriodic() {
      run {
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
      }

      run {
        val leftBumper = controller.getBumper(Hand.kLeft)
        val rightBumper = controller.getBumper(Hand.kRight)
        if (leftBumper)
          lift.raise()
        else if (rightBumper)
          lift.lower()
        else
          lift.stop()     
      }

      run {
        val yButton = controller.getYButton()
        val aButton = controller.getAButton()
        if (yButton)
          wrist.raise()
        else if (aButton)
          wrist.lower()
        else
          wrist.stop()  
      }
      run {
        val leftTrigger: Boolean = (controller.getTriggerAxis(Hand.kLeft) > Speeds.TRIGGER_THRESHOLD)
        val rightTrigger: Boolean = (controller.getTriggerAxis(Hand.kRight) > Speeds.TRIGGER_THRESHOLD)
        if (rightTrigger)
          intake.grab()
        else if (leftTrigger)
          intake.eject()
        else
          intake.stop() 
      }
    }
}
