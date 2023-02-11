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

    companion object{
        @JvmField var ARM_IN = 0.435
        @JvmField var ARM_OUT= 0.65
    }
    init {
        if(opModeType == OpModeType.AUTO) {
            armIn()
        }
    }

    fun armIn(){
        arm.position = ARM_IN
    }

    fun armOut(armOut:Double = ARM_OUT){
        arm.position=armOut
    }
}