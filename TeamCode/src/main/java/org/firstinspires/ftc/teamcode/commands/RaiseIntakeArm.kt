package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

class RaiseIntakeArm(private var intake: IntakeArm, var delay: Int = 0) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        time.reset()
        while (time.milliseconds()<delay){}
        intake.armIn()//reduce to go down
        time.reset()
    }

    override fun isFinished(): Boolean {
        return time.milliseconds() >= 250
    }
}