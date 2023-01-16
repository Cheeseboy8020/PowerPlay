package org.firstinspires.ftc.teamcode.commands
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

class OpenIntakePinch(private var intake: IntakeArm) : CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        intake.open()
    }

    override fun isFinished(): Boolean {
        return true
    }
}