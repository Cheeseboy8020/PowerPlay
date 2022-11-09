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
        const val RESET_POS = 0.5
    }

    //Creates 2 box servos
    var pinch1: Servo
    var pinch2: Servo

    enum class PinchState{
        OPEN, CLOSED
    }

    var state = PinchState.CLOSED

    init {
        this.pinch1 = hardwareMap.get(Servo::class.java, "pinch1")
        this.pinch2 = hardwareMap.get(Servo::class.java, "pinch2")
    }

    fun open(){
        pinch1.position = RESET_POS
        pinch2.position = RESET_POS
        state = PinchState.OPEN
    }

    fun close(){
        pinch1.position = RESET_POS + 0.5
        pinch2.position = RESET_POS + 0.5
        state = PinchState.CLOSED
    }
}