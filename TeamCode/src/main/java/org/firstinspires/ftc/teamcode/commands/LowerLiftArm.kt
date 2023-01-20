package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.LiftArm

class LowerLiftArm(private var lift: LiftArm) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(lift)
    }

    override fun initialize() {
        time.reset()
        lift.armIn()
    }

    override fun isFinished(): Boolean {
        return time.milliseconds() >= 250
    }
}