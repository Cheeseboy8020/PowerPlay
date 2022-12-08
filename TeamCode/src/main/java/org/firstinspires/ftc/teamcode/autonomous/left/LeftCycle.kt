package org.firstinspires.ftc.teamcode.autonomous.left

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.START
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.P1
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.P2
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.P3
import org.firstinspires.ftc.teamcode.autonomous.left.Positions.JUNC
import org.firstinspires.ftc.teamcode.autonomous.AutoBase
import org.firstinspires.ftc.teamcode.autonomous.right.Positions
import org.firstinspires.ftc.teamcode.commands.DropCone
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.commands.LiftGoToPos
import org.firstinspires.ftc.teamcode.commands.PickCone
import org.firstinspires.ftc.teamcode.cv.SignalScanner
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Pinch
import org.firstinspires.ftc.teamcode.util.OpModeType

@Autonomous
class LeftCycle: AutoBase() {
    private lateinit var drive: MecanumDrive // Initialize Drive Variable
    private lateinit var lift: Lift // Initialize Lift Variable
    private lateinit var pinch: Pinch // Initialize Pinch Variable
    private var signalPos: Pose2d = P1 // Initialize Signal
    private var numPos = 1 // Initialize Signal
    private lateinit var scanner:  SignalScanner // Initialize Scanner (CV)

    override fun initialize() {
        telemetry.sendLine("Initializing Subsystems...") // Sends Telemetry Line

        drive = MecanumDrive(hardwareMap) // Assigns Drive Variable
        drive.poseEstimate = START // Sets Drive Pose To Start Pos

        lift = Lift(hardwareMap, telemetry, Lift.Positions.IN_ROBOT, OpModeType.AUTO) // Assigns Lift Variable

        pinch = Pinch(hardwareMap, telemetry) // Assigns Pinch Variable

        scanner = SignalScanner(hardwareMap, telemetry) // Assigns Scanner Variable

        pinch.close() // Closes Pinch To Pick Up Preload Cone

        while(!isStarted){ // Scans CV during the rest of Init
            signalPos = when(scanner.scanBarcode()){
                1-> P1
                2-> P2
                3-> P3
                else -> {
                    telemetry.sendLine("DON'T START")
                    P1
                }
            }// Scans QR Code and Assigns it to signalPos Variable
            telemetry.clear()
            telemetry.sendLine(scanner.scanBarcode().toString()) // Sends Telemetry of Parking Pos
            numPos = scanner.scanBarcode()
        }

        telemetry.sendLine("Generating Trajectories...")
        val goToJunction = drive.trajectorySequenceBuilder(START) // Goes to high junction
            .lineTo(START.vec()+Vector2d(5.0, -5.0))
            .forward(35.0)
            .lineToLinearHeading(JUNC)
            .turn(Math.toRadians(10.0))
            .turn(Math.toRadians(-10.0))
            .build()

        val goToStack = drive.trajectorySequenceBuilder(goToJunction.end()) // Goes to parking positions based CV
            .strafeLeft(13.0)
            .lineToLinearHeading(Positions.STACK)
            .build()

        val goToJunction2 = drive.trajectorySequenceBuilder(goToStack.end()) // Goes to parking positions based CV
            .lineToLinearHeading(Pose2d(Positions.JUNC.vec().x, Positions.JUNC.vec().y+13, Positions.JUNC.heading))
            .strafeRight(13.0)
            .forward(6.0)
            .build()


        val goToParkTemp =
                drive.trajectorySequenceBuilder(goToJunction.end()) // Goes to parking positions based CV
                    .lineToLinearHeading(Pose2d(goToJunction.end().vec()+Vector2d(0.0, 10.0),Math.toRadians((180.0))))
                    .lineTo(signalPos.vec())
                    .turn(Math.toRadians(-90.0))

        val goToPark = when(numPos){
            1->goToParkTemp.strafeRight(3.0).build()
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
            DropCone(pinch), // Drops cone
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.FIVE_STACK, 10, 500),
                FollowTrajectorySequence(drive, goToStack)
            ),
            PickCone(pinch),
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.HIGH),
                FollowTrajectorySequence(drive, goToJunction2)
            ),
            WaitCommand(500),
            DropCone(pinch),
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.FOUR_STACK, 10, 500),
                FollowTrajectorySequence(drive, goToStack)
            ),
            PickCone(pinch),
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.HIGH),
                FollowTrajectorySequence(drive, goToJunction2)
            ),
            WaitCommand(500),
            DropCone(pinch),
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.IN_ROBOT, 10, 500),
                FollowTrajectorySequence(drive, goToPark)
            )
        ))
    }
}