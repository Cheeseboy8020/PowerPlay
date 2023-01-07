package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.util.OpModeType

class Lift(
    hardwareMap: HardwareMap,
    startingPosition: Positions,
    opModeType: OpModeType
) : SubsystemBase() {

    companion object{
        @JvmStatic fun rpmToTicksPerSecond(rpm: Double): Double {
            return rpm * 28*(68.0/13)*(84.0/29)*(84.0/29)/60
        }
    }


    val leftLift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "leftLift")
    val rightLift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rightLift")
    var batteryVoltageSensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()


    enum class Positions(val targetPosition: Int) {
        HIGH(3600),
        MEDIUM(2600),
        LOW(1700),
        GROUND(300),
        IN_ROBOT(0)
    }

    var currentPosition = Positions.IN_ROBOT

    init {
        this.currentPosition = startingPosition
        when(opModeType){
            OpModeType.AUTO -> {
                this.leftLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                this.rightLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                this.leftLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                this.rightLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
            else -> {
                this.leftLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                this.rightLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }
        //this.leftLift.direction = DcMotorSimple.Direction.REVERSE
        this.leftLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.rightLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun setPower(power: Double){
        this.leftLift.power = power*(12/batteryVoltageSensor.voltage)
        this.rightLift.power = power*(12/batteryVoltageSensor.voltage)
    }
}