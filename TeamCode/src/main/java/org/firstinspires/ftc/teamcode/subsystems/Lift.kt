package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Lift(hardwareMap: HardwareMap, private val telemetry: Telemetry? = null, startingPosition: Positions) : SubsystemBase() {
    val lift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "lift")


    enum class Positions(val targetPosition: Int) {
        HIGH(3500),
        MEDIUM(2600),
        LOW(1700),
        GROUND(100),
        IN_ROBOT(10),
        STACK(50),
        STACK_DOWN(-50),
        FIVE_STACK(830),
        FOUR_STACK(700),
        THREE_STACK(300),
        TWO_STACK(200)
    }

    var currentPosition = Positions.IN_ROBOT
    var targetVelo = 0.0


    init {
        this.currentPosition = startingPosition
        this.lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        this.lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        //this.lift.direction = DcMotorSimple.Direction.REVERSE
        this.lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
}