package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

@Config
class IntakeExtension(hardwareMap: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {
    var extLeft: Servo
    var extRight: Servo
    companion object{
        //TODO: Figure these out
        @JvmField var LEFT_IN = 0.42
        @JvmField var RIGHT_IN = 0.58
        const val LEFT_OUT_MAX = 0.01
        @JvmField var LEFT_IN_MIN = 0.42
        const val RIGHT_OUT_MAX = 0.99
        @JvmField var RIGHT_IN_MIN = 0.58
        const val MAX_EXT = 27 // Maximum extension in inches
        // of the intake slide
        const val EXT_OFFSET = 0.05856 * 100.0/2.54 + 6.0

        // Offset from the center of the robot and arm length
        const val EXT_SPEED  = 0.4/750.0 // Rate of extension in servo position per millisecond
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

    init {
        extLeft = hardwareMap.get(Servo::class.java, "extLeft")
        extLeft.direction = Servo.Direction.FORWARD
        extRight = hardwareMap.get(Servo::class.java, "extRight")
    }


    fun retract(){
        extLeft.direction = Servo.Direction.FORWARD
        extLeft.position = LEFT_IN
        extRight.position = RIGHT_IN
        telemetry.addData("pos", extLeft.position)
        telemetry.update()
    }

    fun retractFull(){
        extend(Pair(LEFT_IN_MIN, RIGHT_IN_MIN))
    }

    fun extend(pos: Pair<Double, Double>){
        extLeft.direction = Servo.Direction.FORWARD
        extLeft.position = pos.first
        extRight.position = pos.second
        telemetry.addData("pos", extLeft.position)
        telemetry.update()
    }

}