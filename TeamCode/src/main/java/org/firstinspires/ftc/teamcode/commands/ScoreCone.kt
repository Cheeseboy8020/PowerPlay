package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive

class ScoreCone(intake: Intake, lift: Lift, drive: MecanumDrive, stackPos: Int): SequentialCommandGroup() {
    /*init {
        addCommands(
            ParallelCommandGroup(
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        ExtendIntake(intake, drive.poseEstimate.vec()),
                        LowerIntakeArm(intake, stackPos),
                    ),
                    IntakeGrabCone(intake),
                    ParallelCommandGroup(
                        RetractIntake(intake, drive.poseEstimate.vec()),
                        RaiseIntakeArm(intake, stackPos)
                    )
                ),
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        ExtendLift(lift, drive.poseEstimate.vec()),
                        RaiseLiftArm(lift, stackPos),
                    ),
                    LiftDropCone(lift),
                    ParallelCommandGroup(
                        RetractIntake(intake, drive.poseEstimate.vec()),
                        LowerLiftArm(lift, stackPos)
                    )
                )
            ),
            ParallelCommandGroup(
                DropConeIntake(intake),
                LiftGrabCone(lift)
            )
        )
    }*/
}