package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.LiftArm

class CloseLiftPinch(private var lift: LiftArm) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(lift)
    }

    override fun initialize() {
        time.reset()
    }

    override fun isFinished(): Boolean {
        return time.milliseconds() >= 500
    }
}