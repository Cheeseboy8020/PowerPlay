package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

class Intake(hardwareMap: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {
    var leftPinch: Servo
    var rightPinch: Servo
    var arm: Servo
    var extLeft: Servo
    var extRight: Servo

    enum class PinchState{
        OPEN, CLOSED
    }

    var state = PinchState.CLOSED

    init {
        leftPinch = hardwareMap.get(Servo::class.java, "leftPinch")
        rightPinch = hardwareMap.get(Servo::class.java, "rightPinch")
        arm = hardwareMap.get(Servo::class.java, "intakeArm")
        extLeft = hardwareMap.get(Servo::class.java, "extLeft")
        extRight = hardwareMap.get(Servo::class.java, "extRight")
    }

    fun open(){
        //right
        leftPinch.position = 0.11
        //left
        rightPinch.position = 0.41
        state = PinchState.OPEN
    }

    fun close(){
        //right
        leftPinch.position = 0.04
        //left
        rightPinch.position = 0.46
        state = PinchState.CLOSED
    }
}