package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

class Pinch(hardwareMap: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {
    companion object{
        const val THRESHOLD = 2.0
        const val DROP_TIME_TOP = 1000
        const val DROP_TIME = 1500
        const val RESET_POS_1 = 0.5
        const val RESET_POS_2 = 0.5
    }

    //Creates 2 box servos
    var leftPinch: Servo
    var rightPinch: Servo

    enum class PinchState{
        OPEN, CLOSED
    }

    var state = PinchState.CLOSED

    init {
        this.leftPinch = hardwareMap.get(Servo::class.java, "leftPinch")
        this.rightPinch = hardwareMap.get(Servo::class.java, "rightPinch")
    }

    fun open(){
        //right
        leftPinch.position = 0.45
        //left
        rightPinch.position = 0.75
        state = PinchState.OPEN
    }

    fun close(){
        //right
        leftPinch.position = 0.31
        //left
        rightPinch.position = 0.8
        state = PinchState.CLOSED
    }
}