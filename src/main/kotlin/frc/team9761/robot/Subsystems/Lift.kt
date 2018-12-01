package frc.team9761.robot.Subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team9761.robot.Ports
import frc.team9761.robot.Speeds
import edu.wpi.first.wpilibj.DigitalInput

class Lift:Subsystem("lift") {
    val liftA = WPI_TalonSRX(Ports.LIFT_LEFT_A_CANID)
    val liftB = WPI_TalonSRX(Ports.LIFT_LEFT_B_CANID)
    val liftC = WPI_TalonSRX(Ports.LIFT_RIGHT_A_CANID)
    val liftD = WPI_TalonSRX(Ports.LIFT_RIGHT_B_CANID)
    val liftHeightLimitSwitch = DigitalInput(4)


    override fun initDefaultCommand() {

    }

    fun raise() {
        SmartDashboard.putBoolean("LiftLimit Switch", liftHeightLimitSwitch.get())
        if (liftHeightLimitSwitch.get()) {
            setLiftPower(Speeds.LIFT_RAISE_POWER)
        }
    }

    fun lower() {
        setLiftPower(Speeds.LIFT_LOWER_POWER)
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
        SmartDashboard.putNumber("Talon SRX current A", liftA.getOutputCurrent())
        SmartDashboard.putNumber("Talon SRX current B", liftB.getOutputCurrent())
        SmartDashboard.putNumber("Talon SRX current C", liftC.getOutputCurrent())
        SmartDashboard.putNumber("Talon SRX current D", liftD.getOutputCurrent())
    }
}
