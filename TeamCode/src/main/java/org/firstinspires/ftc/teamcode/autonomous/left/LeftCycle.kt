package org.firstinspires.ftc.teamcode.autonomous.left

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.START
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.P1
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.P2
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.P3
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.STACK
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.JUNC
import org.firstinspires.ftc.teamcode.autonomous.AutoBase
import org.firstinspires.ftc.teamcode.commands.DropCone
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.commands.LiftGoToPos
import org.firstinspires.ftc.teamcode.commands.PickCone
import org.firstinspires.ftc.teamcode.cv.SignalScanner
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.util.OpModeType

@Autonomous(name="LeftCycle")
class LeftCycle: AutoBase() {
    /*
    private lateinit var drive: MecanumDrive // Initialize Drive Variable
    private lateinit var lift: Lift // Initialize Lift Variable
    private lateinit var intake: Intake // Initialize Pinch Variable
    private var signalPos: Pose2d = P1 // Initialize Signal
    private var numPos = 1 // Initialize Signal
    private lateinit var scanner:  SignalScanner // Initialize Scanner (CV)

    override fun initialize() {
        telemetry.sendLine("Initializing Subsystems...") // Sends Telemetry Line

        drive = MecanumDrive(hardwareMap) // Assigns Drive Variable
        drive.poseEstimate = START // Sets Drive Pose To Start Pos

        lift = Lift(hardwareMap, Lift.Positions.IN_ROBOT, OpModeType.AUTO) // Assigns Lift Variable

        intake = Intake(hardwareMap, telemetry) // Assigns Pinch Variable

        scanner = SignalScanner(hardwareMap, telemetry) // Assigns Scanner Variable

        intake.close() // Closes Pinch To Pick Up Preload Cone

        while(!isStarted){ // Scans CV during the rest of Init
            signalPos = when(scanner.scanBarcode()){
                1-> P1
                2-> P2
                3-> P3
                else -> {P1}
            }// Scans QR Code and Assigns it to signalPos Variable
            telemetry.sendLine(scanner.scanBarcode().toString()) // Sends Telemetry of Parking Pos
            numPos = scanner.scanBarcode()
        }

        telemetry.sendLine("Generating Trajectories...")

        val goToJunction = drive.trajectorySequenceBuilder(START) // Goes to high junction
            .lineTo(START.vec()+Vector2d(5.0, -5.0))
            .forward(31.0)
            .lineToLinearHeading(JUNC)
            //.turn(Math.toRadians(7.0))
            //.turn(Math.toRadians(-6.0))
            .forward(6.0)
            .build()

        val goToStack = drive.trajectorySequenceBuilder(goToJunction.end()) // Goes to parking positions based CV
            .strafeRight(13.0)
            .lineToLinearHeading(STACK)
            .forward(5.0)
            .build()

        val goToJunction2 = drive.trajectorySequenceBuilder(goToStack.end()) // Goes to parking positions based CV
            .lineToLinearHeading(Pose2d(JUNC.vec().x, JUNC.vec().y+13, JUNC.heading))
            .strafeLeft(13.0)
            .forward(5.5)
            .build()

        val goToParkTemp =
            drive.trajectorySequenceBuilder(goToJunction.end()) // Goes to parking positions based CV
                .back(5.0)
                .lineToLinearHeading(Pose2d(goToJunction.end().vec()+Vector2d(0.0, 10.0),Math.toRadians((0.0))))
                .lineTo(signalPos.vec())

        val goToPark = when(numPos){
            1->goToParkTemp.build()
            else -> goToParkTemp.build()
        }

        telemetry.sendLine("Scheduling Commands...")

        schedule(SequentialCommandGroup(
            LiftGoToPos(lift, Lift.Positions.GROUND),
            //FollowTrajectorySequence(drive, goToJunction1), // Goes to first position (Corner of tile C1)
            ParallelCommandGroup( // Makes everything in parenthesis run parallel (Goes to high junction and lifts the lift at the same time)
                FollowTrajectorySequence(drive, goToJunction), // Goes to High Junction
                LiftGoToPos(lift, Lift.Positions.HIGH) // Lifts the lift
            ),
            WaitCommand(500),
            DropCone(intake), // Drops cone
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.FIVE_STACK, 10, 1000),
                FollowTrajectorySequence(drive, goToStack)
            ),
            PickCone(intake),
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.HIGH),
                FollowTrajectorySequence(drive, goToJunction2)
            ),
            WaitCommand(500),
            DropCone(intake),
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.FOUR_STACK, 10, 1000),
                FollowTrajectorySequence(drive, goToStack)
            ),
            PickCone(intake),
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.HIGH),
                FollowTrajectorySequence(drive, goToJunction2)
            ),
            WaitCommand(500),
            DropCone(intake),
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.IN_ROBOT, 10, 1000),
                FollowTrajectorySequence(drive, goToPark)
            )
        ))
    }*/
}