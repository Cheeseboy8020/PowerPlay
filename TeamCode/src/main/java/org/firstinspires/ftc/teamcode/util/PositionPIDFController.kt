package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.subsystems.Lift

@Config
class PositionPIDFController(var lift: Lift) {
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
        get() = PROFILED_PID.setpoint.velocity

    var targetVelo = 0.0
        get() = PROFILED_PID.setpoint.velocity

    fun update(measuredPosition: Double, targetAccel: Double): Double {
        val correction = PROFILED_PID.calculate(measuredPosition)
        val feedforward = MOTOR_FF.calculate(PROFILED_PID.setpoint.velocity, targetAccel)
        return correction+feedforward
    }
}