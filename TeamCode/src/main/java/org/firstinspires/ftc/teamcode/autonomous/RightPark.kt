package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.cv.SignalScanner
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer
import org.firstinspires.ftc.teamcode.subsystems.*
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
        intakeArm = IntakeArm(hardwareMap, telemetry)
        drive = MecanumDrive(hardwareMap)
        drive.poseEstimate = Positions.A2
        intake = IntakeExtension(hardwareMap, telemetry)
        lift = Lift(hardwareMap, Lift.Positions.IN_ROBOT, OpModeType.AUTO)
        scanner = SignalScanner(hardwareMap, telemetry)

        while(!isStarted)
        {
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
            telemetry.update()
        }
        if (isStopRequested){
            T265Localizer.slamera!!.stop()
        }
        telemetry.sendLine("Generated Trajectories...")
        val goToCycle = drive.trajectorySequenceBuilder(Positions.A2)
            .lineToLinearHeading(Pose2d(-36.0, 24.0, Math.toRadians(270.0)))
            .lineToLinearHeading(Pose2d(-45.3, 5.3, Math.toRadians(346.0)))
            .build()

        val goToPark = drive.trajectorySequenceBuilder(goToCycle.end())
            .lineToLinearHeading(pos)
            .build()

        telemetry.sendLine("Ready To Start...")
        waitForStart()
        telemetry.sendLine("Scheduling Commands...")
        schedule(
            SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("Program Started!")
                    telemetry.update()
                }),
                ParallelCommandGroup(
                    FollowTrajectorySequence(drive, goToCycle),
                    LiftGoToPos(lift, Lift.Positions.HIGH),
                    RaiseLiftArm(liftArm)
                ),
                OpenLiftPinch(liftArm),
                ParallelCommandGroup(
                    FollowTrajectorySequence(drive, goToPark),
                    LiftGoToPos(lift, Lift.Positions.IN_ROBOT),
                    LowerLiftArm(liftArm)
                ),
                InstantCommand({T265Localizer.slamera!!.stop()})

        )
        )
    }
}