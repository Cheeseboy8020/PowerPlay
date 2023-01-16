package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.OpModeType

@Config
class IntakeArm(hardwareMap: HardwareMap, val telemetry: Telemetry, opModeType: OpModeType
) : SubsystemBase() {
    var leftPinch: Servo
    var rightPinch: Servo
    var arm: Servo

    enum class PinchState{
        OPEN, CLOSED
    }

    var state = PinchState.CLOSED

    init {
        leftPinch = hardwareMap.get(Servo::class.java, "leftPinch")
        rightPinch = hardwareMap.get(Servo::class.java, "rightPinch")
        arm = hardwareMap.get(Servo::class.java, "intakeArm")
        if (opModeType == OpModeType.AUTO) {
            open()
            armIn()
        }
    }

    fun open(){
        leftPinch.position = 0.3
        rightPinch.position=0.7
    }

    fun close(){
        leftPinch.position=0.5
        rightPinch.position=0.5
    }
    fun armOut(stackHeight: Int){
        when(stackHeight) {
            1->arm.position = 0.54 //1 stack
            2->arm.position = 0.5366//2 stack
            3->arm.position = 0.5333 //3 stack
            4->arm.position = 0.53 //4 stack
            5->arm.position = 0.52 //5 stack
        }
    }
    fun armIn(){
        arm.position=0.45
    }

}