package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.kinematics.Kinematics.calculateMotorFeedforward

class PositionPIDFController(pid: PIDCoefficients?, private var kV: Double = 0.0, private var kA: Double = 0.0, private var kStatic: Double = 0.0) {
    var controller: PIDFController?
    private var lastPosition = Double.NaN
    private var lastVelocity = Double.NaN

    init{
        controller = PIDFController(pid!!)
        reset()
    }


    fun update(measuredPosition: Double, measuredVelocity: Double): Double {
        if (lastPosition.isNaN()) {
            lastPosition = measuredPosition
        }
        if (lastVelocity.isNaN()) {
            lastVelocity = measuredVelocity
        }
        val correction = controller!!.update(measuredPosition, measuredVelocity)
        val feedforward = calculateMotorFeedforward(
            controller!!.targetPosition, controller!!.targetVelocity, kV, kA, kStatic
        )
        return correction + feedforward
    }

    fun reset() {
        controller!!.reset()
        lastPosition = Double.NaN
        lastVelocity = Double.NaN
    }
}