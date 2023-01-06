package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.util.PositionPIDFController
import kotlin.math.abs

@Config
class ExtendLift(val lift: Lift): CommandBase(){

    val liftController = PositionPIDFController()
    val time = ElapsedTime()

    init{
        addRequirements(lift)
    }

    override fun initialize() {
        //once
        liftController.targetPos =
        time.reset()
    }

    //Run repeatedly while the command is active
    override fun execute() {
        //Update the lift power with the
    }

    override fun isFinished(): Boolean {
        //End if the lift position is within the tolerance
        return false
    }

    override fun end(interrupted: Boolean) {
    }
}