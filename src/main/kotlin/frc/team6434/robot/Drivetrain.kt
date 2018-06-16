package frc.team6434.robot


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class Drivetrain {
    val leftA = WPI_TalonSRX(Ports.LEFT_A_CANID)
    val leftB = WPI_TalonSRX(Ports.LEFT_B_CANID)
    val rightA = WPI_TalonSRX(Ports.RIGHT_A_CANID)
    val rightB = WPI_TalonSRX(Ports.RIGHT_B_CANID)
    
    init {
        leftB.follow(leftA)
        rightB.follow(rightA)
    }

    fun setPower(leftPower: Double, rightPower: Double) {
      SmartDashboard.putNumber("leftPower", leftPower)
      SmartDashboard.putNumber("rightPower", rightPower)
      leftA.set(leftPower)
      rightA.set(rightPower)
    }
}
