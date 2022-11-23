package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Lift
import kotlin.math.abs

@Config
class LiftGoToPos(private val lift: Lift, private val pos: Lift.Positions) : CommandBase() {
    @JvmField val coeffs = PIDCoefficients(0.0, 0.0, 0.0)
    val tolerance = 10.0
    val liftController = PIDFController(coeffs, 0.0, 0.0, 0.1)

    init {
        liftController.setOutputBounds(-0.8, 1.0)
        addRequirements(lift)
    }

    override fun initialize() {
        lift.lift.power = 0.0
        liftController.reset()
        liftController.targetPosition = pos.targetPosition.toDouble()
    }

    override fun execute() {
        val liftPosition = lift.lift.currentPosition;
        //Update the lift power with the controller
        lift.lift.power = liftController.update(liftPosition.toDouble())
        lift.targetVelo = liftController.targetVelocity
    }

    override fun isFinished(): Boolean {
        return abs(lift.lift.currentPosition - pos.targetPosition) < tolerance
    }

    override fun end(interrupted: Boolean) {
        lift.lift.power = 0.0
    }
}