package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.Encoder
import org.firstinspires.ftc.teamcode.util.OpModeType

@Config
class IntakeExtension(hardwareMap: HardwareMap, val telemetry: Telemetry, opModeType: OpModeType) : SubsystemBase() {
    var extLeft: Servo
    var extRight: Servo
    var encoder: Encoder
    companion object{
        //TODO: Figure these out
        @JvmField var LEFT_IN = 0.5
        @JvmField var RIGHT_IN = 0.5
        @JvmField var LEFT_IN_MIN = 0.45
        @JvmField var RIGHT_IN_MIN = 0.55
        const val EXT_SPEED  = 0.4/750.0 // Rate of extension in servo position per millisecond
    }

    init {
        extLeft = hardwareMap.get(Servo::class.java, "extLeft")
        extRight = hardwareMap.get(Servo::class.java, "extRight")
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


    fun retract(){
        extLeft.position = LEFT_IN
        extRight.position = RIGHT_IN
        telemetry.addData("pos", extLeft.position)
        telemetry.update()
    }

    fun retractFull(){
        extend(Pair(LEFT_IN_MIN, RIGHT_IN_MIN))
    }

    fun extend(pos: Pair<Double, Double>){
        extLeft.position = pos.first
        extRight.position = pos.second
        telemetry.addData("pos", extLeft.position)
        telemetry.update()
    }

}