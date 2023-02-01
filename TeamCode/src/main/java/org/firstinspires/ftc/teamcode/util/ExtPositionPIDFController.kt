package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.subsystems.Lift

@Config
class ExtPositionPIDFController() {
    companion object {
        @JvmField var kp = 0.0
        @JvmField var ki = 0.0
        @JvmField var kd = 0.0
    }

    var PROFILED_PID = ProfiledPIDController(kp, ki, kd, TrapezoidProfile.Constraints((((60/0.09)/360)*8192), ((60/0.09)/360)*8192))

    var targetPos = 0.0
        set(value) {
            field = value
            PROFILED_PID.setGoal(value)
        }
        get() = field

    var targetVelo = 0.0
        get() = PROFILED_PID.setpoint.velocity


    fun update(measuredPosition: Double): Double {
        return PROFILED_PID.calculate(measuredPosition)
    }

    fun updateCoeffs(){
        PROFILED_PID.d = kd
        PROFILED_PID.i = ki
        PROFILED_PID.p = kp
    }
}