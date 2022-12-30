package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.kinematics.Kinematics.calculateMotorFeedforward
import com.qualcomm.robotcore.util.MovingStatistics
import kotlin.math.abs

class VelocityPIDFController(pid: PIDCoefficients?, private var kV: Double = 0.0, private var kA: Double = 0.0, private var kStatic: Double = 0.0) {
    private var controller: PIDFController? = null
    private var accelSamples: MovingStatistics? = null
    private var lastPosition = Double.NaN
    private var lastVelocity = Double.NaN


    init {
        controller = PIDFController(pid!!)
        accelSamples = MovingStatistics(ACCEL_SAMPLES)
        reset()
    }

    fun setTargetAcceleration(acceleration: Double) {
        controller!!.targetVelocity = acceleration
    }

    fun setTargetVelocity(velocity: Double) {
        controller!!.targetPosition = velocity
    }

    private fun calculateAccel(measuredPosition: Double, measuredVelocity: Double): Double {
        val dx = measuredPosition - lastPosition
        if (dx != 0.0 && abs(measuredVelocity - lastVelocity) > VELOCITY_EPSILON) {
            val accel =
                (measuredVelocity * measuredVelocity - lastVelocity * lastVelocity) / (2.0 * dx)
            lastPosition = measuredPosition
            lastVelocity = measuredVelocity
            accelSamples!!.add(accel)
        } else {
            accelSamples!!.add(0.0)
        }
        return accelSamples!!.mean
    }

    fun update(measuredPosition: Double, measuredVelocity: Double): Double {
        if (lastPosition.isNaN()) {
            lastPosition = measuredPosition
        }
        if (lastVelocity.isNaN()) {
            lastVelocity = measuredVelocity
        }
        val accel = calculateAccel(measuredPosition, measuredVelocity)
        val correction = controller!!.update(measuredVelocity, accel)
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

    companion object {
        private const val ACCEL_SAMPLES = 3
        private const val VELOCITY_EPSILON = 20 + 1e-6
    }
}