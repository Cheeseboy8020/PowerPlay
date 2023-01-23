package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence


class AutoPositionBlue(var drive: MecanumDrive, var gamepad1: GamepadEx): CommandBase() {
    init {
        addRequirements(drive)
    }

    override fun initialize() {
        var trajectorySequence = drive.trajectorySequenceBuilder(drive.poseEstimate)
            .lineToLinearHeading(Pose2d(0.0, 44.0, Math.toRadians(270.0)))
            .build()
        drive.followTrajectorySequenceAsync(trajectorySequence)
    }

    override fun execute() {
        drive.update()
    }

    override fun end(interrupted: Boolean) {
        GamepadDrive(drive, { gamepad1.leftY }, { gamepad1.leftX }, { gamepad1.rightX }).schedule()
        drive.stop()
    }

    override fun isFinished(): Boolean {
        return Thread.currentThread().isInterrupted || !drive.isBusy
    }
}