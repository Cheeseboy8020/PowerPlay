package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Lift

class LiftGoToPos(private val lift: Lift, private val pos: Lift.Positions) : CommandBase() {
    init {
        addRequirements(lift)
    }

    override fun initialize() {
        lift.currentPosition = pos
    }

    override fun execute() {
        if (lift.lift.currentPosition < lift.currentPosition.targetPosition)
            lift.lift.power = 1.0
        else
            lift.lift.power = -1.0
    }

    override fun isFinished(): Boolean {
        return approximatelyEqual(lift.lift.currentPosition, lift.currentPosition.targetPosition, 5.0)
    }

    override fun end(interrupted: Boolean) {
        lift.lift.power = 0.0
    }

    fun approximatelyEqual(
        desiredValue: Int,
        actualValue: Int,
        tolerancePercentage: Double
    ): Boolean {
        val diff = Math.abs(desiredValue - actualValue) //  1000 - 950  = 50
        val tolerance = tolerancePercentage / 100 * desiredValue //  20/100*1000 = 200
        return diff < tolerance //  50<200      = true
    }
}