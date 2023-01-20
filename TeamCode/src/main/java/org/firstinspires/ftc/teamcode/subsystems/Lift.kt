package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.util.OpModeType

@Config
class Lift(
    hardwareMap: HardwareMap,
    startingPosition: Positions,
    opModeType: OpModeType
) : SubsystemBase() {

    companion object {
        @JvmStatic
        fun rpmToTicksPerSecond(rpm: Double): Double {
            return rpm * 28 * (68.0 / 13) * (84.0 / 29) * (84.0 / 29) / 60
        }
        @JvmField var HIGH = 1300
        @JvmField var HIGH_AUTO = 775
    }

    val leftLift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "leftLift")
    val rightLift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rightLift")
    val batteryVoltageSensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()


    enum class Positions(val targetPosition: Int) {
        HIGH(Lift.HIGH),
        HIGH_AUTO(Lift.HIGH_AUTO),
        MEDIUM(950),
        LOW(350),
        IN_ROBOT(20)
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
        this.leftLift.direction = DcMotorSimple.Direction.REVERSE
        this.leftLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.rightLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun setPower(power: Double){
        this.leftLift.power = power
        this.rightLift.power = power
    }
}