package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Lift(hardwareMap: HardwareMap, private val telemetry: Telemetry? = null, startingPosition: Positions) : SubsystemBase() {
    val lift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "lift")


    enum class Positions(val targetPosition: Int) {
        HIGH(3600),
        MEDIUM(2700),
        LOW(1700),
        IN_ROBOT(5),
    }

    var currentPosition = Positions.IN_ROBOT


    init {
        this.currentPosition = startingPosition
        this.lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        this.lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        //this.lift.direction = DcMotorSimple.Direction.REVERSE
        this.lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
}