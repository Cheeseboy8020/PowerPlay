package org.firstinspires.ftc.teamcode.commands
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.LiftArm

class OpenLiftPinch(private var lift: LiftArm) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(lift)
    }

    override fun initialize() {
        time.reset()
        lift.open()
    }

    override fun isFinished(): Boolean {
        return time.milliseconds() >= 250
    }
}