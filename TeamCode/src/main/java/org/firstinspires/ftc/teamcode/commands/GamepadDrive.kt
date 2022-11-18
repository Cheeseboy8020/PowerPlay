package org.firstinspires.ftc.teamcode.commands

import android.util.Log
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
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
        Log.w("GamepadDrive", "Executing")
        drive.drive.leftFront.power = (leftY.asDouble * 0.75) + (leftX.asDouble * 0.75) + (rightX.asDouble * 0.75)
        drive.drive.leftRear.power = (leftY.asDouble * 0.75) - (leftX.asDouble * 0.75) + (rightX.asDouble * 0.75)
        drive.drive.rightFront.power = (leftY.asDouble * 0.75) - (leftX.asDouble * 0.75) - (rightX.asDouble * 0.75)
        drive.drive.rightRear.power = (leftY.asDouble * 0.75) + (leftX.asDouble * 0.75) - (rightX.asDouble * 0.75)
    }

    init {
        this.drive = drive
        this.leftX = leftX
        this.leftY = leftY
        this.rightX = rightX
        addRequirements(drive)
    }
}