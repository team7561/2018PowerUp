package frc.team9761.robot.Subsystems

import edu.wpi.first.wpilibj.Talon
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team9761.robot.Ports
import frc.team9761.robot.Speeds

class Intake {

    private val intakeLeft = Talon(Ports.INTAKE_LEFT_CHANNEL)
    private val intakeRight = Talon(Ports.INTAKE_RIGHT_CHANNEL)

    fun grab() {
        setIntakePower(Speeds.IN_POWER)
    }

    fun eject() {
        setIntakePower(Speeds.OUT_POWER)
    }

    fun stop() {
        setIntakePower(0.0)
    }

    private fun setIntakePower(intakePower: Double) {
        SmartDashboard.putNumber("intakePower", intakePower)
        intakeLeft.set(intakePower)
        intakeRight.set(-intakePower)
    }
}
