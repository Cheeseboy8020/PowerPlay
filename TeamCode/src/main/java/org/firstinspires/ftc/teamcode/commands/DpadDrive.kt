package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import com.arcrobotics.ftclib.util.Direction
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive


class DpadDrive(var drive: MecanumDrive, var dir: Direction): CommandBase() {
    lateinit var initialPose: Pose2d
    lateinit var transController: PIDFController
    lateinit var headingController: PIDFController
    init {
        addRequirements(drive)
    }

    override fun initialize() {
        initialPose = drive.poseEstimate
        transController = PIDFController(SampleMecanumDrive.TRANSLATIONAL_PID)
        headingController = PIDFController(SampleMecanumDrive.HEADING_PID)
        transController.targetPosition =
    }

    override fun execute() {
        when(dir) {
            Direction.FORWARD -> drive.setWeightedDrivePower(Pose2d(1.0, , ))
            Direction.BACKWARDS -> drive.setWeightedDrivePower(Pose2d(0.0, -0.5, 0.0))
            Direction.LEFT -> drive.setWeightedDrivePower(Pose2d(-0.5, 0.0, 0.0))
            Direction.RIGHT -> drive.setWeightedDrivePower(Pose2d(0.5, 0.0, 0.0))
        }
        drive.update()
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    override fun isFinished(): Boolean {
        return Thread.currentThread().isInterrupted || !drive.isBusy
    }
}