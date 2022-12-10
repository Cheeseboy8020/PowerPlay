package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Lift
import kotlin.math.abs

@Config
class LiftGoToPos(val lift: Lift, val pos: Lift.Positions, val tolerance: Int = 10, val delay: Int = 0) : CommandBase() {
    companion object{
        @JvmField
        var coefficients = PIDCoefficients(0.0335, 0.0, 0.00025)
    }

    val liftController = PIDFController(coefficients, 0.0, 0.0, 0.1)
    val time = ElapsedTime()


    init{
        liftController.setOutputBounds(-0.8, 1.0)
        addRequirements(lift)
    }

    override fun initialize() {
        //once
        lift.lift.power = 0.0
        liftController.reset()
        liftController.targetPosition = pos.targetPosition.toDouble()
        time.reset()
    }

    //Run repeatedly while the command is active
    override fun execute() {
        //Update the lift power with the
        if (time.milliseconds() < delay)
        {
            lift.lift.power = 0.1
        }
        else {
            lift.lift.power = liftController.update(lift.lift.currentPosition.toDouble())
        }
    }

    override fun isFinished(): Boolean {
        //End if the lift position is within the tolerance
        return abs(lift.lift.currentPosition - pos.targetPosition) < tolerance
    }

    override fun end(interrupted: Boolean) {
        when(pos) {
            Lift.Positions.IN_ROBOT->lift.lift.power=0.0

            else -> {lift.lift.power=0.1}
        }
    }
}