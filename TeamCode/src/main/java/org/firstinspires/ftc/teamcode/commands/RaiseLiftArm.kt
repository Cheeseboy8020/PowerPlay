package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.LiftArm

class RaiseLiftArm(private var lift: LiftArm, var armPos: Double=0.6) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(lift)
    }

    override fun initialize() {
        time.reset()
        lift.armOut(armPos)
    }

    override fun isFinished(): Boolean {
        return time.milliseconds() >= 250
    }
}