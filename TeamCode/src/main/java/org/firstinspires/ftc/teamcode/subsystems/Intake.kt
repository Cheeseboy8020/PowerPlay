package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.autonomous.right.Right

class Intake(hardwareMap: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {
    var leftPinch: Servo
    var rightPinch: Servo
    var arm: Servo
    var extLeft: Servo
    var extRight: Servo

    enum class PinchState{
        OPEN, CLOSED
    }

    companion object{
        const val LEFT_OUT_MAX = 1.0
        const val LEFT_IN_MIN = 0.3
        const val RIGHT_OUT_MAX = 0.0
        const val RIGHT_IN_MIN = 0.7
        const val MAX_EXT = 18
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

    fun extend(){
        extLeft.position = 0.0
        extRight.position = 1.0
    }

    fun calcPos(robotPos: Vector2d, ): Pair<Double, Double>{

        return Pair(0.0, 0.0)
    }
}