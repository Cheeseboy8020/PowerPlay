package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

class LowerIntakeArm(private var intake: IntakeArm, var stackHeight: Int) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        time.reset()
        intake.armOut(stackHeight)
    }
    override fun isFinished(): Boolean {
        return time.milliseconds() >= 350
    }
}