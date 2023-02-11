package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.hardware.ams.AMSColorSensor.Wait
import org.firstinspires.ftc.teamcode.subsystems.*

@Config
class ScoreConeTimed(intakeArm: IntakeArm, liftArm: LiftArm, intakeExtension: IntakeExtension, lift: Lift, stackHeight: Int, extPos: Double, liftHeight: Lift.Positions, turret: LiftTurret, turretAngle: Double = 0.0, armPos: Double = 0.65, dropDelay: Long = 500, wait: Long = 0): SequentialCommandGroup() {
    companion object{
        @JvmField var TURRET_DELAY = 250
        @JvmField var INTAKE_ARM_DELAY = 750

    }
    init {
        addCommands(
            WaitCommand(wait),
            ResetTurret(turret),
            OpenIntakePinch(intakeArm),
            ParallelCommandGroup(
                LiftGoToPos(lift, liftHeight),
                RaiseLiftArm(liftArm, armPos),
                SequentialCommandGroup(
                    WaitCommand(TURRET_DELAY.toLong()),
                    LiftTurretPosition(turret,turretAngle)
                ),
                LowerIntakeArm(intakeArm, stackHeight),
                ExtendIntakeTime(intakeExtension, extPos),
            ),
            WaitCommand(dropDelay),
            CloseIntakePinch(intakeArm),
            ParallelCommandGroup(
                ResetTurret(turret),
                LowerLiftArm(liftArm),
                LiftGoToPos(lift, Lift.Positions.IN_ROBOT),
                SequentialCommandGroup(
                    WaitCommand(INTAKE_ARM_DELAY.toLong()),
                    RaiseIntakeArm(intakeArm)
                ),
                RetractIntake(intakeExtension)
            ),
        )
    }
}