package org.firstinspires.ftc.teamcode.autonomous.right

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
class RightPark: AutoBase() {
    private lateinit var drive: MecanumDrive
    private lateinit var intake: IntakeExtension
    private lateinit var lift: Lift
    private lateinit var liftArm: LiftArm
    private lateinit var intakeArm: IntakeArm
    private lateinit var scanner: SignalScanner
    private lateinit var pos: Pose2d

    companion object{
        @JvmField var ARM_POS = 0.6
        @JvmField var EXT_POS = 0.365
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

        //Decrease y to go towards the tv
        //Decreasing x goes towards the wall without anything and increasing x to goes to the wall where we work
        val goToPark = drive.trajectorySequenceBuilder(Positions.START)
            .lineTo(Positions.START.vec()+Vector2d(6.0, -6.0)) //Initial movement to go between junctions
            .lineTo(Vector2d(-35.0, 18.0))
            .lineTo(pos.vec()) // Goes to cycle position
            .build()

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
                InstantCommand({intake.retractFull()}),
                FollowTrajectorySequence(drive, goToPark),
                InstantCommand({ PoseStorage.pose = drive.poseEstimate}),
                InstantCommand({slamera!!.stop()})
        )
        )
    }
}