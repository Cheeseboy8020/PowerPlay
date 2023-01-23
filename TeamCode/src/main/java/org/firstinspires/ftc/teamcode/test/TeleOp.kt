package org.firstinspires.ftc.teamcode.test

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.alphago.agDistanceLocalization.geometry.Pose
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.PerpetualCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.autonomous.AutoBase.Companion.sendLine
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage.pose
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.util.OpModeType


@TeleOp
class TeleOp: CommandOpMode() {

    override fun initialize() {
        val gamepad1 = GamepadEx(gamepad1)
        val gamepad2 = GamepadEx(gamepad2)


        val lift = Lift(hardwareMap, Lift.Positions.IN_ROBOT, OpModeType.AUTO)
        val drive = MecanumDrive(hardwareMap)
        drive.poseEstimate = pose
        val intake = IntakeExtension(hardwareMap, telemetry)
        intake.retractFull()
        val intakeArm = IntakeArm(hardwareMap, telemetry, OpModeType.AUTO)
        val liftArm = LiftArm(hardwareMap, OpModeType.TELEOP)


        val scoreCone = ScoreCone(intakeArm, liftArm, intake, lift, 1, 0.38, Lift.Positions.HIGH)
        val scoreConeMid = ScoreCone(intakeArm, liftArm, intake, lift, 1, 0.365, Lift.Positions.MEDIUM)
        val autoPos = AutoPositionBlue(drive, gamepad1)
        Log.w("TeleOp", "Initialized Lift")
        telemetry.sendLine("Initialized Robot")
        //Auto position - Done
        //One button to flip arm and raise lift (lowers lift also) - Done
        //Lowers arm and lowers lift - Done
        //Full retract extending - Done
        //Tranfer retract - Done
        //Intake pinch/unpinch
        //Intake arm/bring to transfer position
        //Intake Arm out
        //Emergency stop - Done
        /*gamepad1.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(autoPos)

        gamepad1.getGamepadButton(GamepadKeys.Button.Y)
            .cancelWhenPressed(autoPos)*/


        gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(LiftGoToPos(lift, when(lift.currentPosition){
                Lift.Positions.HIGH->Lift.Positions.MEDIUM
                Lift.Positions.MEDIUM -> Lift.Positions.LOW
                Lift.Positions.LOW -> Lift.Positions.IN_ROBOT
                else -> {Lift.Positions.HIGH}
            }))
            .whenPressed(RaiseLiftArm(liftArm))
        gamepad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.IN_ROBOT))
            .whenPressed(LowerLiftArm(liftArm))
        gamepad2.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(scoreCone)
        gamepad2.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(scoreConeMid)
        gamepad2.getGamepadButton(GamepadKeys.Button.BACK)
            .cancelWhenPressed(scoreCone)
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(RaiseIntakeArm(intakeArm))
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(LowerIntakeArm(intakeArm, 1))
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(CloseIntakePinch(intakeArm))
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(OpenIntakePinch(intakeArm))

        Trigger{gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0 || gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0}
            .whileActiveContinuous(InstantCommand({lift.setPower((gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))/2.0)}, lift))
            .whenInactive(InstantCommand({lift.setPower(0.0)}, lift))

        schedule(GamepadDrive(drive, { gamepad1.leftY }, { gamepad1.leftX }, { gamepad1.rightX }))
        register(lift, intake, intakeArm, liftArm, drive)
    }
}