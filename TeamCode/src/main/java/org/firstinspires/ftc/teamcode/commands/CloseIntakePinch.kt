package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

class CloseIntakePinch(private var intake: IntakeArm) : CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        intake.leftPinch.position=0.5
        intake.rightPinch.position=0.5
    }

    override fun isFinished(): Boolean {
        return true
    }
}