package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile

@Config
class PositionPIDFController() {
    companion object {
        @JvmField var MOTOR_PID = PIDController(0.0, 0.0, 0.0)
        @JvmField var MOTOR_FF = ElevatorFeedforward(0.0, 0.0, 0.0, 0.0)
    }

    var PROFILED_PID = ProfiledPIDController(MOTOR_PID.p, MOTOR_PID.i, MOTOR_PID.d, TrapezoidProfile.Constraints(5960*28/60.0, 5960*28/60.0))

    var targetPos = 0.0
        set(value) {
            field = value
            PROFILED_PID.setGoal(value)
        }
    var targetVelo = 0.0
    init {
        MOTOR_PID.reset()
    }

    fun update(measuredPosition: Double, measuredVelocity: Double): Double {
        val correction = MOTOR_PID.calculate(measuredPosition)
        val feedforward = MOTOR_FF.calculate(targetPos, targetVelo)
        return correction+feedforward
    }

    fun reset() {
        MOTOR_PID.reset()
    }
}