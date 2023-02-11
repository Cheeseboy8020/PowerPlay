package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive

@Config
class AutoPosition(var drive: MecanumDrive, var gamepad1: GamepadEx): CommandBase() {
    companion object{
        @JvmField var x = 5.0
        @JvmField var y = 35.0
    }
    init {
        addRequirements(drive)
    }

    override fun initialize() {
        val trajectorySequence = drive.trajectorySequenceBuilder(drive.poseEstimate)
            .lineToLinearHeading(Pose2d(x, y, Math.toRadians(270.0)))
            .build()
        drive.followTrajectorySequenceAsync(trajectorySequence)
    }

    override fun execute() {
        drive.update()
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    override fun isFinished(): Boolean {
        return Thread.currentThread().isInterrupted || !drive.isBusy
    }
}