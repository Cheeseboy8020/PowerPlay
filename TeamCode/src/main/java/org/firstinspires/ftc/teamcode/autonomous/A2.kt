package org.firstinspires.ftc.teamcode.autonomous

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.Positions.A2
import org.firstinspires.ftc.teamcode.autonomous.Positions.P1
import org.firstinspires.ftc.teamcode.commands.DropCone
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.cv.SignalScanner
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Pinch

@Autonomous
class A2: AutoBase() {
    private lateinit var drive: MecanumDrive // Initialize Drive Variable
    private lateinit var lift: Lift // Initialize Lift Variable
    private lateinit var pinch: Pinch // Initialize Pinch Variable
    private var signalPos: Vector2d = P1 // Initialize Signal
    private lateinit var scanner:  SignalScanner // Initialize Scanner (CV)

    override fun initialize() {
        telemetry.sendLine("Initializing Subsystems...") // Sends Telemetry Line

        drive = MecanumDrive(hardwareMap) // Assigns Drive Variable
        drive.poseEstimate = A2 // Sets Drive Pose To Start Pos

        lift = Lift(hardwareMap, telemetry, Lift.Positions.IN_ROBOT) // Assigns Lift Variable

        pinch = Pinch(hardwareMap, telemetry) // Assigns Pinch Variable

        scanner = SignalScanner(hardwareMap, telemetry) // Assigns Scanner Variable

        pinch.close() // Closes Pinch To Pick Up Preload Cone

        while(!isStarted){ // Scans CV during the rest of Init
            signalPos = scanner.scanBarcode().first // Scans QR Code and Assigns it to signalPos Variable
            telemetry.sendLine(scanner.scanBarcode().second.toString()) // Sends Telemetry of Parking Pos
            Log.d("Pos", signalPos.toString())
        }

        telemetry.sendLine("Generating Trajectories...")

        val goToJunction1 = drive.trajectorySequenceBuilder(A2) // Goes to position before raising lift
            .strafeRight(20.0)
            .lineTo(Vector2d(-48.0, 17.0))
            .build()

        val goToJunction2 = drive.trajectorySequenceBuilder(goToJunction1.end()) // Goes to high junction
            .splineTo(Positions.X2.vec(), Positions.X2.heading)
            .build()

        val goToPark =
                drive.trajectorySequenceBuilder(goToJunction2.end()) // Goes to parking positions based CV
                    .lineTo(signalPos)
                    .build()

        telemetry.sendLine("Scheduling Commands...")

        schedule(SequentialCommandGroup(
            LiftGoToPos(lift, Lift.Positions.GROUND), // Lifts the lift so the cone doesn't scrape on the ground
            FollowTrajectorySequence(drive, goToJunction1), // Goes to first position (Corner of tile C1)
            ParallelCommandGroup( // Makes everything in parenthesis run parallel (Goes to high junction and lifts the lift at the same time)
                FollowTrajectorySequence(drive, goToJunction2), // Goes to High Junction
                LiftGoToPos(lift, Lift.Positions.HIGH) // Lifts the lift
            ),
            DropCone(pinch), // Drops cone
            ParallelCommandGroup( // Lowers lift while going to parking position at the time.
                LiftGoToPos(lift, Lift.Positions.IN_ROBOT),
                FollowTrajectorySequence(drive, goToPark)
            )
        ))
    }
}