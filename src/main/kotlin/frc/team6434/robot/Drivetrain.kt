package frc.team6434.robot


import edu.wpi.first.wpilibj.VictorSP
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class Drivetrain {


    val leftA = VictorSP(Ports.DRIVE_LEFT_A_CHANNEL)
    val leftB = VictorSP(Ports.DRIVE_LEFT_B_CHANNEL)
    val rightA = VictorSP(Ports.DRIVE_RIGHT_A_CHANNEL)
    val rightB = VictorSP(Ports.DRIVE_RIGHT_B_CHANNEL)
    
    
    fun setPower(leftPower: Double, rightPower: Double) {
      SmartDashboard.putNumber("leftPower", leftPower)
      SmartDashboard.putNumber("rightPower", rightPower)
      leftA.set(leftPower)
      leftB.set(leftPower)
      rightA.set(rightPower)
      rightB.set(rightPower)
    }
}
