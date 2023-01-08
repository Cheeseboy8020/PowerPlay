package org.firstinspires.ftc.teamcode.autonomous.right

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.AutoBase

@Autonomous
class Right: AutoBase() {
    /*private lateinit var drive: MecanumDrive // Initialize Drive Variable
    private lateinit var lift: Lift // Initialize Lift Variable
    private lateinit var intake: Intake // Initialize Pinch Variable
    private var signalPos: Pose2d = P1 // Initialize Signal
    private var numPos = 1 // Initialize Signal
    private lateinit var scanner:  SignalScanner // Initialize Scanner (CV)

    override fun initialize() {
        telemetry.sendLine("Initializing Subsystems...") // Sends Telemetry Line

        drive = MecanumDrive(hardwareMap) // Assigns Drive Variable
        drive.poseEstimate = START // Sets Drive Pose To Start Pos

        lift = Lift(hardwareMap, telemetry, Lift.Positions.IN_ROBOT, OpModeType.AUTO) // Assigns Lift Variable

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
            .forward(30.0)
            .lineToLinearHeading(JUNC)
            .turn(Math.toRadians(7.0))
            .turn(Math.toRadians(-7.0))
            .build()


        val goToParkTemp =
                drive.trajectorySequenceBuilder(goToJunction.end()) // Goes to parking positions based CV
                    .lineToLinearHeading(Pose2d(goToJunction.end().vec()+Vector2d(0.0, 10.0),Math.toRadians((0.0))))
                    .lineTo(signalPos.vec())
                    .turn(Math.toRadians(90.0))

        val goToPark = when(numPos){
            3->goToParkTemp.strafeLeft(5.0).build()
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
                LiftGoToPos(lift, Lift.Positions.IN_ROBOT, 10, 500),
                FollowTrajectorySequence(drive, goToPark)
            ),
        ))
    }*/
}