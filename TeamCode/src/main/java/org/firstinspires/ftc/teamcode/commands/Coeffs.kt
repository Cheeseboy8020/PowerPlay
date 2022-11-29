package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients

@Config
object Coefficients {
    @JvmField var kP = 0.0335
    @JvmField var kI = 0.0
    @JvmField var kD = 0.00025
}