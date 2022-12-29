package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.commands.LiftGoToPos
import org.firstinspires.ftc.teamcode.util.OpModeType

class Lift(
    hardwareMap: HardwareMap,
    startingPosition: Positions,
    opModeType: OpModeType
) : SubsystemBase() {


    val leftLift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "leftLift")
    val rightLift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rightLift")


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
}