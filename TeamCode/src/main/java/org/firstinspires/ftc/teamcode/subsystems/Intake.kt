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
        //TODO: Figure these out
        const val LEFT_OUT_MAX = 1.0
        const val LEFT_IN_MIN = 0.3
        const val RIGHT_OUT_MAX = 0.0
        const val RIGHT_IN_MIN = 0.7
        const val MAX_EXT = 0.69 * 100.0/2.54 // Maximum extension in inches
        // of the intake slide
        const val EXT_OFFSET = 0.05856 * 100.0/2.54 // Offset from the center of the robot
        const val EXT_SPEED  = 0.7/1000.0 // Rate of extension in servo position per millisecond
        fun calcPos(robotPos: Vector2d, goal: Vector2d): Pair<Double, Double>{
            val dist = robotPos.distTo(goal)-EXT_OFFSET
            if(dist >= MAX_EXT) {
                return Pair(LEFT_OUT_MAX, RIGHT_OUT_MAX)
            }
            val leftPos = LEFT_OUT_MAX - (LEFT_OUT_MAX - LEFT_IN_MIN) * (dist / MAX_EXT)
            val rightPos = RIGHT_OUT_MAX + (RIGHT_IN_MIN - RIGHT_OUT_MAX) * (dist / MAX_EXT)
            return Pair(leftPos, rightPos)
        }
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

    fun retract(){
        extLeft.position = 0.4
        extRight.position = 0.8
    }

    fun extend(pos: Pair<Double, Double>){
        extLeft.position = pos.first
        extRight.position = pos.second
    }


}