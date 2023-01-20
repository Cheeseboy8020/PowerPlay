package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

class CloseIntakePinch(private var intake: IntakeArm) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        time.reset()
        intake.leftPinch.position=0.5
        intake.rightPinch.position=0.5
    }

    override fun isFinished(): Boolean {
        return time.milliseconds() >= 250
    }
}