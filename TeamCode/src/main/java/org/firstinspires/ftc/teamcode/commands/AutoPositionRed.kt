package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence


class AutoPositionRed(var drive: MecanumDrive): CommandBase() {
    var trajectorySequence = drive.trajectorySequenceBuilder(drive.poseEstimate)
        .lineToLinearHeading(Pose2d(0.0, 38.0, Math.toRadians(90.0)))
        .build()
    init {
        addRequirements(drive)
    }

    override fun initialize() {
        drive.followTrajectorySequenceAsync(trajectorySequence)
    }

    override fun execute() {
        drive.update()
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            drive.stop()
        }
    }

    override fun isFinished(): Boolean {
        return Thread.currentThread().isInterrupted || !drive.isBusy
    }
}