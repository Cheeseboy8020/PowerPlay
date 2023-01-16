package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

class RaiseIntakeArm(private var intake: IntakeArm) : CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        intake.arm.position=0.45//reduce to go down
    }

    override fun isFinished(): Boolean {
        return true
    }
}