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
    private val telemetry: Telemetry? = null,
    startingPosition: Positions,
    opModeType: OpModeType
) : SubsystemBase() {
    companion object{
        @JvmField
        var liftPosition = 0
    }


    val lift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "lift")


    enum class Positions(val targetPosition: Int) {
        HIGH(3500),
        MEDIUM(2600),
        LOW(1700),
        GROUND(300),
        IN_ROBOT(20),
        FIVE_STACK(775),
        FOUR_STACK(675),
        THREE_STACK(700),
        TWO_STACK(675)
    }

    var currentPosition = Positions.IN_ROBOT
    var targetVelo = 0.0


    init {
        this.currentPosition = startingPosition
        when(opModeType){
            OpModeType.AUTO -> {
                this.lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                this.lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
            else -> {
                this.lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }
        //this.lift.direction = DcMotorSimple.Direction.REVERSE
        this.lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
}