package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystems.*

class ScoreCone(intakeArm: IntakeArm, liftArm: LiftArm, intakeExtension: IntakeExtension, lift: Lift,drive: MecanumDrive, stackVec: Vector2d, stackHeight: Int): SequentialCommandGroup() {
    init {
        addCommands(
            ParallelCommandGroup(
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        ExtendIntake(intakeExtension, drive.poseEstimate.vec(), stackVec),
                        LowerIntakeArm(intakeArm, stackHeight),
                    ),
                    CloseIntakePinch(intakeArm),
                    ParallelCommandGroup(
                        RetractIntake(intakeExtension),
                        RaiseIntakeArm(intakeArm)
                    )
                ),
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        ExtendLift(lift, Lift.Positions.HIGH),
                        RaiseLiftArm(liftArm),
                    ),
                    CloseLiftPinch(liftArm),
                    ParallelCommandGroup(
                        RetractIntake(intakeExtension),
                        LowerLiftArm(liftArm)
                    )
                )
            ),
            ParallelCommandGroup(
                OpenIntakePinch(intakeArm),
                CloseLiftPinch(liftArm)
            )
        )
    }
}