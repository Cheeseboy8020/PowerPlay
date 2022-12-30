package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.kinematics.Kinematics.calculateMotorFeedforward

class PositionPIDFController(pid: PIDCoefficients?, private var kV: Double = 0.0, private var kA: Double = 0.0, private var kStatic: Double = 0.0) {
    var controller: PIDFController?
    val MOTOR_VELO_PID = PIDCoefficients(0.0, 0.0, 0.0)
    val velokV = 0.0
    val velokA = 0.0
    val velokStatic = 0.0
    val veloController = VelocityPIDFController(MOTOR_VELO_PID, velokV, velokA, velokStatic)

    init{
        controller = PIDFController(pid!!)
        reset()
    }


    fun update(measuredPosition: Double, measuredVelocity: Double): Double {
        val correction = controller!!.update(measuredPosition, measuredVelocity)
        val feedforward = calculateMotorFeedforward(
            controller!!.targetPosition, controller!!.targetVelocity, kV, kA, kStatic
        )
        veloController.setTargetVelocity(correction+feedforward)
        return veloController.update(measuredPosition, measuredVelocity)
    }

    fun reset() {
        controller!!.reset()
    }
}