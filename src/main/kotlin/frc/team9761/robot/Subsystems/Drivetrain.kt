package frc.team9761.robot.Subsystems

import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.VictorSP
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team9761.robot.Ports

class Drivetrain {

    val leftA = VictorSP(Ports.DRIVE_LEFT_A_CHANNEL)
    val leftB = VictorSP(Ports.DRIVE_LEFT_B_CHANNEL)
    val rightA = VictorSP(Ports.DRIVE_RIGHT_A_CHANNEL)
    val rightB = VictorSP(Ports.DRIVE_RIGHT_B_CHANNEL)

    val leftEncoder = Encoder(Ports.ENCODER_LEFT_A_CHANNEL, Ports.ENCODER_LEFT_B_CHANNEL)
    val rightEncoder = Encoder(Ports.ENCODER_RIGHT_A_CHANNEL, Ports.ENCODER_RIGHT_B_CHANNEL, true) // Invert right encoder

    fun setPower(leftPower: Double, rightPower: Double) {
        SmartDashboard.putNumber("leftPower", leftPower)
        SmartDashboard.putNumber("rightPower", rightPower)
        leftA.set(leftPower)
        leftB.set(leftPower)
        rightA.set(rightPower)
        rightB.set(rightPower)
    }

    fun getDistance(): Double {
        var distance: Double = 0.5 * (leftEncoder.get() + rightEncoder.get())
        SmartDashboard.putNumber("Left Encoder", leftEncoder.get().toDouble())
        SmartDashboard.putNumber("Right Encoder", rightEncoder.get().toDouble())
        SmartDashboard.putNumber("Distance", distance)
        return distance
    }

    fun resetEncoders() {
        leftEncoder.reset()
        rightEncoder.reset()
    }
}
