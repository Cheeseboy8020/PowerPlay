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
        @JvmField var HIGH = 600
        @JvmField var HIGH_ANGLE = 800
        @JvmField var MEDIUM = 300
        @JvmField var LOW = 350
        @JvmField var IN_ROBOT = 20
    }

    val leftLift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "leftLift")
    val rightLift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rightLift")
    val batteryVoltageSensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()


    enum class Positions(val targetPosition: Int) {
        HIGH(Lift.HIGH),
        HIGH_ANGLE(Lift.HIGH_ANGLE),
        MEDIUM(Lift.MEDIUM),
        LOW(Lift.LOW),
        IN_ROBOT(Lift.IN_ROBOT)
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
        this.rightLift.direction = DcMotorSimple.Direction.REVERSE
        this.leftLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.rightLift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun setPower(power: Double){
        this.leftLift.power = power
        this.rightLift.power = power
    }

    val position: Int
    get() = leftLift.currentPosition
}