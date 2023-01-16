package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.util.OpModeType

class LiftArm(
    hardwareMap: HardwareMap,
    opModeType: OpModeType
) : SubsystemBase() {

    val arm = hardwareMap.get(Servo::class.java, "liftArm")
    val leftPinch = hardwareMap.get(Servo::class.java, "liftLeftPinch")
    val rightPinch = hardwareMap.get(Servo::class.java, "liftRightPinch")
    init {
        close()
        armIn()
    }

    fun open(){
        leftPinch.position = 0.4
        rightPinch.position = 0.6

    }
    fun close(){
        leftPinch.position = 0.5
        rightPinch.position = 0.5
    }

    fun armIn(){
        arm.position = 0.45
    }

    fun armOut(){
        arm.position=0.6
    }
}