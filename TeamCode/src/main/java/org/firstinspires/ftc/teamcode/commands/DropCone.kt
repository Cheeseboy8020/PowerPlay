package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Intake

class DropCone(private val intake: Intake) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        time.reset()
        intake.open()
    }


    override fun isFinished(): Boolean {
        return time.milliseconds() > 700
    }
}