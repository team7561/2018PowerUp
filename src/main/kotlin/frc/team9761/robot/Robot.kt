package frc.team9761.robot

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.GenericHID.Hand
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team9761.robot.Drivers.ADIS16448_IMU
import frc.team9761.robot.Subsystems.Drivetrain
import frc.team9761.robot.Subsystems.Intake
import frc.team9761.robot.Subsystems.Lift
import frc.team9761.robot.Subsystems.Wrist

object TheRobot {
    lateinit var myRobot: Robot
    fun setInstance(robot: Robot) { myRobot = robot }
    fun getInstance(): Robot = myRobot
}
class Robot : IterativeRobot() {
    companion object {
        lateinit var controller: XboxController
        lateinit var drivetrain: Drivetrain
        lateinit var lift: Lift
        lateinit var wrist: Wrist
        lateinit var intake: Intake
        lateinit var gyro: ADIS16448_IMU // ADXRS453_Gyro
    }
    var distance: Double = 0.0
    var position: AnalogInput = AnalogInput(1)
    var cylinder: DoubleSolenoid = DoubleSolenoid(0, 1)

    override fun robotInit() {
        TheRobot.setInstance(this)

        println("Hello Illawarra 9761")

        controller = XboxController(Ports.XBOX_CONTROLLER_PORT)
        CameraServer.getInstance().startAutomaticCapture()

        drivetrain = Drivetrain()
        lift = Lift()
        wrist = Wrist()
        intake = Intake()
        gyro = ADIS16448_IMU()
        gyro.calibrate()

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
        gyro.reset()
        drivetrain.resetEncoders()
    }

    override fun autonomousPeriodic() {
        val currentTime = System.currentTimeMillis()
        val elapsedTime = currentTime - startTime
        var drivePower = if (elapsedTime < Speeds.CROSS_LINE_DURATION) Speeds.CROSS_LINE_POWER else 0.0
        drivetrain.setPower(drivePower, drivePower)

        if (elapsedTime < Speeds.LIFT_RAISE_DURATION)
            lift.raise()
        else {
            lift.stop()
            wrist.release()
        }
    }

    override fun teleopInit() {
        drivetrain.resetEncoders()
        Scheduler.getInstance().run()
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
            //drivetrain.setPower(0.0, 0.0)
            distance = drivetrain.getDistance()
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
            if (rightTrigger) {
                intake.grab()
                cylinder.set(DoubleSolenoid.Value.kForward)
            } else if (leftTrigger) {
                intake.eject()
                cylinder.set(DoubleSolenoid.Value.kReverse)
            } else {
                intake.stop()
                cylinder.set(DoubleSolenoid.Value.kOff)
            }
        }
        run {
            // arduino.sendMessage("test")
            SmartDashboard.putNumber("Angle X", gyro.angleX)
            SmartDashboard.putNumber("Angle Y", gyro.angleY)
            SmartDashboard.putNumber("Angle Z", gyro.angleZ)
            SmartDashboard.putNumber("Distance", distance)
            SmartDashboard.putNumber("Position", position.voltage)
            //SmartDashboard.putString("Value", arduinoSerial.readString())

        }
    }
}
