package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.right.RightPositions
import org.firstinspires.ftc.teamcode.autonomous.right.RightPositions.START
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.cv.SignalScanner
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer.Companion.slamera
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.util.OpModeType

@Config
@Autonomous
class RightPark: AutoBase() {
    private lateinit var drive: MecanumDrive
    private lateinit var intake: IntakeExtension
    private lateinit var lift: Lift
    private lateinit var liftArm: LiftArm
    private lateinit var turret: LiftTurret
    private lateinit var intakeArm: IntakeArm
    private lateinit var scanner: SignalScanner
    private lateinit var pos: Vector2d

    companion object{
        @JvmField var ARM_POS = 0.6
        @JvmField var EXT_POS = 0.365
    }

    override fun initialize() {
        telemetry.sendLine("Initializing Subsystems...")
        liftArm = LiftArm(hardwareMap, OpModeType.AUTO)
        intakeArm = IntakeArm(hardwareMap, telemetry, OpModeType.AUTO)
        turret = LiftTurret(hardwareMap, OpModeType.AUTO)
        drive = MecanumDrive(hardwareMap)

        intake = IntakeExtension(hardwareMap, telemetry, OpModeType.AUTO)
        lift = Lift(hardwareMap, Lift.Positions.IN_ROBOT, OpModeType.AUTO)
        scanner = SignalScanner(hardwareMap, telemetry)

        while(!isStarted)
        {
            drive.poseEstimate = RightPositions.START
            pos = when(scanner.scanBarcode()){
                1 -> RightPositions.P1
                2 -> RightPositions.P2
                3 -> RightPositions.P3
                else ->
                {
                    telemetry.sendLine("Don't Start!")
                    RightPositions.P1
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
        drive.poseEstimate = START
        telemetry.addData("x", drive.poseEstimate.x)
        telemetry.addData("y", drive.poseEstimate.y)
        telemetry.addData("heading", Math.toDegrees(drive.poseEstimate.heading))
        telemetry.update()
        telemetry.sendLine("Generated Trajectories...")

        val goToCycle =
            drive.trajectorySequenceBuilder(START)
                .lineTo(START.vec()+Vector2d(6.0, -6.0))
                .lineTo(Vector2d(-33.0, 37.0))
                .build()

        val goToPark =
            drive.trajectorySequenceBuilder(goToCycle.end())
                .lineTo(pos)
                .build()

        drive.poseEstimate = goToCycle.start()
        telemetry.sendLine("Ready To Start...")
        waitForStart()
        telemetry.sendLine("Scheduling Commands...")
        schedule(
            InstantCommand({intake.retractFull()}),
            InstantCommand({turret.pos = 0.3}),
                FollowTrajectorySequence(drive, goToPark),
            InstantCommand({intakeArm.armIn()}),
            SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("Program Started!")
                    telemetry.update()
                }),
                FollowTrajectorySequence(drive, goToCycle),
                InstantCommand({intake.retractFull()}),
                FollowTrajectorySequence(drive, goToPark),
                InstantCommand({ PoseStorage.pose = drive.poseEstimate}),
                InstantCommand({slamera!!.stop()})
            )
        )
    }
}