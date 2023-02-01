package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.OpModeType

@Config
class IntakeArm(hardwareMap: HardwareMap, val telemetry: Telemetry, opModeType: OpModeType
) : SubsystemBase() {
    var pinch: Servo
    var arm: Servo

    enum class PinchState{
        OPEN, CLOSED
    }

    companion object{
        @JvmField var STACK_1_POS = 0.5375
        @JvmField var STACK_2_POS =  0.53//2 stack
        @JvmField var STACK_3_POS = 0.5225 //3 stack
        @JvmField var STACK_4_POS = 0.5175 //4 stack
        @JvmField var STACK_5_POS = 0.5125 //5 stack
        @JvmField var ARM_IN = 0.43
    }
    var state = PinchState.CLOSED

    init {
        pinch = hardwareMap.get(Servo::class.java, "intakePinch")
        arm = hardwareMap.get(Servo::class.java, "intakeArm")
        if (opModeType == OpModeType.AUTO) {
            open()
            armIn()
        }
    }

    fun open(){
        pinch.position = 0.43
    }

    fun close(){
        pinch.position=0.5

    }
    fun armOut(stackHeight: Int){
        when(stackHeight) {
            1->arm.position = STACK_1_POS //1 stack
            2->arm.position = STACK_2_POS //2 stack
            3->arm.position = STACK_3_POS //3 stack
            4->arm.position = STACK_4_POS //4 stack
            5->arm.position = STACK_5_POS //5 stack
        }
    }
    fun armIn(){
        arm.position= ARM_IN
    }

}