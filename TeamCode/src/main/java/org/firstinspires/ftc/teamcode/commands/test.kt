package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward

fun main() {
    var MOTOR_PID = PIDController(0.001, 0.0, 0.0)
    var MOTOR_FF = ElevatorFeedforward(0.0, 0.0, 0.00001, 0.0)
    MOTOR_PID.setPoint = 20000.0
    var voltage = 12.0
    println(MOTOR_PID.calculate(300.0)+MOTOR_FF.calculate(1000.0, 50.0)*(12/voltage))
    println(5960*28/60.0)
}