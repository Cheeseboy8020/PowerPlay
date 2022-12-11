package org.firstinspires.ftc.teamcode.commands

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
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
        drive.setWeightedDrivePower(
            Pose2d(
                leftY.asDouble,
                -leftX.asDouble,
                -rightX.asDouble
            )
        )
    }

    init {
        this.drive = drive
        this.leftX = leftX
        this.leftY = leftY
        this.rightX = rightX
        addRequirements(drive)
    }
}