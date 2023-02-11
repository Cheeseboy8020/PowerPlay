package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

class RaiseIntakeArm(private var intake: IntakeArm) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        time.reset()
        intake.armIn()
    }

    override fun isFinished(): Boolean {
        return time.milliseconds() >= 350
    }
}