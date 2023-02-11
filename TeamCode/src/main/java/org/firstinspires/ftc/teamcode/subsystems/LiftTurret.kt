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

    companion object{
        @JvmField var TURRET_RESET = 0.3
    }

    val turret = hardwareMap.get(Servo::class.java, "turret")
    val turretEncoder = hardwareMap.get(AnalogInput::class.java, "turretEncoder")
    init {
        if(opModeType == OpModeType.AUTO) {
        }
    }

    var pos: Double = 1-(turretEncoder.voltage/3.3)
        get() = 1-(turretEncoder.voltage/3.3)
        set(value) {
        turret.position = value
        field = value
    }
}