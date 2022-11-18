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
    var pinch1: Servo
    var pinch2: Servo

    enum class PinchState{
        OPEN, CLOSED
    }

    var state = PinchState.CLOSED

    init {
        this.pinch1 = hardwareMap.get(Servo::class.java, "leftPinch")
        this.pinch2 = hardwareMap.get(Servo::class.java, "rightPinch")
    }

    fun open(){
        pinch1.position = 0.12
        pinch2.position = 0.20
        state = PinchState.OPEN
    }

    fun close(){
        pinch1.position = 0.15
        pinch2.position = 0.17
        state = PinchState.CLOSED
    }
}