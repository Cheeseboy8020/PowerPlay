package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.util.OpModeType

@Config
class LiftTurret(
    hardwareMap: HardwareMap,
    opModeType: OpModeType
) : SubsystemBase() {

    val turret = hardwareMap.get(Servo::class.java, "turret")
    val turretEncoder = hardwareMap.get(AnalogInput::class.java, "turretEncoder")

    companion object{
        @JvmField var ARM_IN = 0.44
        @JvmField var ARM_OUT= 0.6
    }
    init {
        if(opModeType == OpModeType.AUTO) {
            setAngle(0.0)
        }
    }

    fun setAngle(angle: Double){
        turret.position = angle/355
    }

    fun getPos(): Double{
        return 3.3/turretEncoder.voltage
    }

    fun getAngle(): Double{
        return getPos()*355
    }
}