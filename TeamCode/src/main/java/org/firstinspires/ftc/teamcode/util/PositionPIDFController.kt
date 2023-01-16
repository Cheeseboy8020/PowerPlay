package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.LiftArm

@Config
class PositionPIDFController(var lift: Lift) {
    companion object {
        @JvmField var kp = 0.03
        @JvmField var ki = 0.0
        @JvmField var kd = 0.0006
        @JvmField var ks = 0.0
        @JvmField var kg = 0.17
        @JvmField var kv = 0.00005
        @JvmField var ka = 0.000
    }

    var PROFILED_PID = ProfiledPIDController(kp, ki, kd, TrapezoidProfile.Constraints(2000.0, 500.0))

    var targetPos = 0.0
        set(value) {
            field = value
            PROFILED_PID.setGoal(value)
        }
        get() = field

    var targetVelo = 0.0
        get() = PROFILED_PID.setpoint.velocity

    var MOTOR_FF = ElevatorFeedforward(ks, kg, kv, ka)

    fun update(measuredPosition: Double, targetAccel: Double): Double {
        val correction = PROFILED_PID.calculate(measuredPosition)
        val feedforward = MOTOR_FF.calculate(targetVelo, targetAccel) * (12/lift.batteryVoltageSensor.voltage)
        return correction+feedforward
    }

    fun updateCoeffs(){
        MOTOR_FF = ElevatorFeedforward(ks, kg, kv, ka)
        PROFILED_PID.d = kd
        PROFILED_PID.i = ki
        PROFILED_PID.p = kp
    }
}