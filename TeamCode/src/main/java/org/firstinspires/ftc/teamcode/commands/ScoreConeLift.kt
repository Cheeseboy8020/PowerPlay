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
class ScoreConeLift(liftArm: LiftArm, lift: Lift, liftHeight: Lift.Positions, armPos:Double=0.6, turret: LiftTurret, turretAngle: Double = 0.0, wait: Long = 0): SequentialCommandGroup() {
    init {
        addCommands(
            WaitCommand(wait),
            ParallelCommandGroup(
                LiftGoToPos(lift, liftHeight),
                RaiseLiftArm(liftArm, armPos),
                SequentialCommandGroup(
                WaitCommand(ScoreConeAutoLift.TURRET_DELAY.toLong()),
                LiftTurretPosition(turret,turretAngle)
            ),
            ),
            WaitCommand(250),
            LowerLiftArm(liftArm),
            LiftGoToPos(lift, Lift.Positions.IN_ROBOT),
        )
    }
}