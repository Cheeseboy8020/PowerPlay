package org.firstinspires.ftc.teamcode.autonomous.right

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.AutoBase
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.autonomous.right.Positions.DELIVER
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.cv.SignalScanner
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer.Companion.slamera
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.util.OpModeType

@Autonomous
class RightPark: AutoBase() {
    private lateinit var drive: MecanumDrive
    private lateinit var intake: IntakeExtension
    private lateinit var lift: Lift
    private lateinit var liftArm: LiftArm
    private lateinit var intakeArm: IntakeArm
    private lateinit var scanner: SignalScanner
    private lateinit var pos: Pose2d


    override fun initialize() {
        telemetry.sendLine("Initializing Subsystems...")
        liftArm = LiftArm(hardwareMap, OpModeType.AUTO)
        intakeArm = IntakeArm(hardwareMap, telemetry, OpModeType.AUTO)
        drive = MecanumDrive(hardwareMap)

        intake = IntakeExtension(hardwareMap, telemetry)
        lift = Lift(hardwareMap, Lift.Positions.IN_ROBOT, OpModeType.AUTO)
        scanner = SignalScanner(hardwareMap, telemetry)

        while(!isStarted)
        {
            drive.poseEstimate = Positions.START
            pos = when(scanner.scanBarcode()){
                1 -> Positions.P1
                2 -> Positions.P2
                3 -> Positions.P3
                else ->
                {
                    telemetry.sendLine("Don't Start!")
                    Positions.P1
                }
            }
            telemetry.clear()
            telemetry.addData("Position", scanner.scanBarcode().toString())
            telemetry.addData("x", drive.poseEstimate.x)
            telemetry.addData("y", drive.poseEstimate.y)
            telemetry.addData("heading", Math.toDegrees(drive.poseEstimate.heading))
            telemetry.update()
        }
        if (isStopRequested){
            slamera!!.stop()
        }
        drive.poseEstimate = Positions.START
        telemetry.addData("x", drive.poseEstimate.x)
        telemetry.addData("y", drive.poseEstimate.y)
        telemetry.addData("heading", Math.toDegrees(drive.poseEstimate.heading))
        telemetry.update()
        telemetry.sendLine("Generated Trajectories...")
        val goToCycle = drive.trajectorySequenceBuilder(Positions.START)
            .lineTo(Positions.START.vec()+Vector2d(5.0, -5.0))
            .lineToLinearHeading(Pose2d(-36.5, 24.5, Math.toRadians(270.0)))
            .lineToLinearHeading(DELIVER)
            .build()

        val back = drive.trajectorySequenceBuilder(goToCycle.end())
            .back(2.0)
            //.lineToLinearHeading(Pose2d(-42.0, 8.5, Math.toRadians(346.0)))
            .build()
        val goToPark: TrajectorySequence = when(pos){
            Positions.P1 -> drive.trajectorySequenceBuilder(back.end()).turn(Math.toRadians(-230.0)).lineToLinearHeading(pos).build()
            else -> drive.trajectorySequenceBuilder(back.end()).lineToLinearHeading(pos).build()
        }

        telemetry.sendLine("Ready To Start...")
        waitForStart()
        telemetry.sendLine("Scheduling Commands...")
        schedule(
            InstantCommand({intake.retractFull()}),
            InstantCommand({drive.poseEstimate= Positions.START }),
            SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("Program Started!")
                    telemetry.update()
                }),
                FollowTrajectorySequence(drive, goToCycle),
                ParallelCommandGroup(
                    LiftGoToPos(lift, Lift.Positions.HIGH),
                    RaiseLiftArm(liftArm)
                ),
                OpenLiftPinch(liftArm),
                FollowTrajectorySequence(drive, back),
                ParallelCommandGroup(
                    LiftGoToPos(lift, Lift.Positions.IN_ROBOT),
                    LowerLiftArm(liftArm)
                ),
                FollowTrajectorySequence(drive, goToPark),
                InstantCommand({ PoseStorage.pose = drive.poseEstimate}),
                InstantCommand({slamera!!.stop()})
            )
        )
    }
}