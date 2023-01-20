package org.firstinspires.ftc.teamcode.autonomous.left

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.AutoBase
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.cv.SignalScanner
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer.Companion.slamera
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.util.OpModeType

@Config
@Autonomous
class LeftCycle: AutoBase() {
    private lateinit var drive: MecanumDrive
    private lateinit var intake: IntakeExtension
    private lateinit var lift: Lift
    private lateinit var liftArm: LiftArm
    private lateinit var intakeArm: IntakeArm
    private lateinit var scanner: SignalScanner
    private lateinit var pos: Pose2d

    companion object{
        @JvmField var ARM_POS = 0.6
        @JvmField var EXT_POS = 0.275
    }

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
            .lineTo(Positions.START.vec()+Vector2d(-6.0, -6.0))
            .lineToLinearHeading(Pose2d(32.0, 24.0, Math.toRadians(270.0)))
            .lineToLinearHeading(Pose2d(38.0, 5.5, Math.toRadians(196.0)))
            .build()

        val goToPark = if(pos== Positions.P3){
            drive.trajectorySequenceBuilder(goToCycle.end())
                .strafeRight(12.0)
                .lineTo(pos.vec())
                .build()
        }
        else {
            drive.trajectorySequenceBuilder(goToCycle.end())
                .lineTo(pos.vec())
                .build()
        }

        telemetry.sendLine("Ready To Start...")
        waitForStart()
        telemetry.sendLine("Scheduling Commands...")
        schedule(
            InstantCommand({intake.retractFull()}),
            SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("Program Started!")
                    telemetry.update()
                }),
                FollowTrajectorySequence(drive, goToCycle),
                ScoreCone(intakeArm, liftArm, intake, lift, 5, EXT_POS, Lift.Positions.HIGH_AUTO, ARM_POS),
                ScoreCone(intakeArm, liftArm, intake, lift, 4, EXT_POS +0.005, Lift.Positions.HIGH_AUTO, ARM_POS, Pair(0.33, 0.67), 350),
                ScoreCone(intakeArm, liftArm, intake, lift, 3, EXT_POS +0.005, Lift.Positions.HIGH_AUTO, ARM_POS, Pair(0.33, 0.67), 350),
                ScoreCone(intakeArm, liftArm, intake, lift, 2, EXT_POS +0.008, Lift.Positions.HIGH_AUTO, ARM_POS, Pair(0.33, 0.67), 350),
                ScoreCone(intakeArm, liftArm, intake, lift, 1, EXT_POS +0.007, Lift.Positions.HIGH_AUTO, ARM_POS, Pair(0.33, 0.67), 350),
                ScoreConeLift(liftArm, lift, Lift.Positions.HIGH_AUTO, ARM_POS, 250),
                InstantCommand({ PoseStorage.pose = drive.poseEstimate}),
                FollowTrajectorySequence(drive, goToPark),
                InstantCommand({slamera!!.stop()})
        )
        )
    }
}