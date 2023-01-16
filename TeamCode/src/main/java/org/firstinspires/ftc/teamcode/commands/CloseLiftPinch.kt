package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.LiftArm

class CloseLiftPinch(private var lift: LiftArm) : CommandBase() {
    init {
        addRequirements(lift)
    }

    override fun initialize() {
        lift.close()
    }

    override fun isFinished(): Boolean {
        return true
    }
}