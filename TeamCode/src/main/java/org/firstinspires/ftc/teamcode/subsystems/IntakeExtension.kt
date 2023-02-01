package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.Encoder
import org.firstinspires.ftc.teamcode.util.OpModeType

@Config
class IntakeExtension(hardwareMap: HardwareMap, val telemetry: Telemetry, opModeType: OpModeType) : SubsystemBase() {
    var extLeft: CRServo
    var extRight: CRServo
    var encoder: Encoder
    companion object{
        //TODO: Figure these out
        @JvmField var LEFT_IN = 0.42
        @JvmField var RIGHT_IN = 0.58
        const val LEFT_OUT_MAX = 0.01
        @JvmField var LEFT_IN_MIN = 0.42
        const val RIGHT_OUT_MAX = 0.99
        @JvmField var RIGHT_IN_MIN = 0.58
    }

    init {
        extLeft = hardwareMap.get(CRServo::class.java, "extLeft")
        extRight = hardwareMap.get(CRServo::class.java, "extRight")
        extRight.direction = DcMotorSimple.Direction.REVERSE
        val motor = hardwareMap.get(DcMotorEx::class.java, "encoder")
        when(opModeType){
            OpModeType.AUTO -> {
                motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
            else->{motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER}
        }
        encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "encoder"))
        encoder.direction = Encoder.Direction.REVERSE
    }

    var position = 0.0
    get() {
        return encoder.currentPosition.toDouble()
    }

    var power = 0.0
    get() = extLeft.power
    set(value) {
        field = value
        extLeft.power = value
        extRight.power = value
    }
}