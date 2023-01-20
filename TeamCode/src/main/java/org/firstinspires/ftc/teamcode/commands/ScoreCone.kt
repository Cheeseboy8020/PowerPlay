package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.hardware.ams.AMSColorSensor.Wait
import org.firstinspires.ftc.teamcode.subsystems.*

@Config
class ScoreCone(intakeArm: IntakeArm, liftArm: LiftArm, intakeExtension: IntakeExtension, lift: Lift, stackHeight: Int, extPos: Double, liftHeight: Lift.Positions, armPos:Double=0.6, retractPos: Pair<Double, Double> = Pair(
    IntakeExtension.LEFT_IN,
    IntakeExtension.RIGHT_IN
), wait: Long = 0): SequentialCommandGroup() {
    init {
        addCommands(
            WaitCommand(wait),
            ParallelCommandGroup(
                OpenIntakePinch(intakeArm),
                CloseLiftPinch(liftArm)
            ),
            ParallelCommandGroup(
                ExtendIntake(intakeExtension, extPos),
                LowerIntakeArm(intakeArm, stackHeight),
                LiftGoToPos(lift, liftHeight),
                RaiseLiftArm(liftArm, armPos)
            ),
            OpenLiftPinch(liftArm),
            WaitCommand(50),
            CloseIntakePinch(intakeArm),
            ParallelCommandGroup(
                RaiseIntakeArm(intakeArm),
                LowerLiftArm(liftArm)
            ),
            LiftGoToPos(lift, Lift.Positions.IN_ROBOT),
            WaitCommand(100),
            RetractIntake(intakeExtension, 100, retractPos)
        )
    }
}