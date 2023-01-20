package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.util.OpModeType

@Config
class LiftArm(
    hardwareMap: HardwareMap,
    opModeType: OpModeType
) : SubsystemBase() {

    val arm = hardwareMap.get(Servo::class.java, "liftArm")
    val leftPinch = hardwareMap.get(Servo::class.java, "liftLeftPinch")
    val rightPinch = hardwareMap.get(Servo::class.java, "liftRightPinch")

    companion object{
        @JvmField var PINCH_CLOSE = 0.47
        @JvmField var PINCH_OPEN = 0.4
        @JvmField var ARM_IN = 0.44
        @JvmField var ARM_OUT= 0.6
    }
    init {
        if(opModeType == OpModeType.AUTO) {
            close()
            armIn()
        }
    }

    fun open(){
        leftPinch.position = PINCH_OPEN
        rightPinch.position = 1.0-PINCH_OPEN

    }
    fun close(){
        leftPinch.position = PINCH_CLOSE
        rightPinch.position = 1.0- PINCH_CLOSE
    }

    fun armIn(){
        arm.position = ARM_IN
    }

    fun armOut(armOut:Double = ARM_OUT){
        arm.position=armOut
    }
}