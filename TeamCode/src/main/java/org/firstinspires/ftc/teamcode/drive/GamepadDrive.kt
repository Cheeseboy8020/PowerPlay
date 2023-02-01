package org.firstinspires.ftc.teamcode.drive

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import java.util.function.DoubleSupplier

class GamepadDrive(
    drive: MecanumDrive, leftY: DoubleSupplier,
    leftX: DoubleSupplier, rightX: DoubleSupplier
) : CommandBase() {
    private val drive: MecanumDrive
    private val leftY: DoubleSupplier
    private val leftX: DoubleSupplier
    private val rightX: DoubleSupplier
    override fun execute() {
        drive.drive(cubeInput(leftY.asDouble, 0.3), cubeInput(leftX.asDouble, 0.3), cubeInput(rightX.asDouble, 0.3))
    }

    init {
        this.drive = drive
        this.leftX = leftX
        this.leftY = leftY
        this.rightX = rightX
        addRequirements(drive)
    }

    fun cubeInput(input: Double, factor: Double): Double {
        val t = factor * Math.pow(input, 3.0)
        val r = input * (1 - factor)
        return t + r
    }
}