package org.firstinspires.ftc.teamcode.test

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.alphago.agDistanceLocalization.geometry.Pose
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.util.OpModeType


@TeleOp
class TeleOp: CommandOpMode() {

    override fun initialize() {

        val gamepad1 = GamepadEx(gamepad1)
        val gamepad2 = GamepadEx(gamepad2)


        val lift = Lift(hardwareMap, telemetry, Lift.Positions.IN_ROBOT, OpModeType.TELEOP)
        val pinch = Pinch(hardwareMap, telemetry)
        val drive = MecanumDrive(hardwareMap)
        Log.w("TeleOp", "Initialized Lift")


        gamepad2.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.LOW))

        gamepad2.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.IN_ROBOT))

        gamepad2.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.HIGH))

        gamepad2.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.MEDIUM))

        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.FIVE_STACK))

        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.FOUR_STACK))

        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.THREE_STACK))

        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.TWO_STACK))

        gamepad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(InstantCommand(pinch::open, pinch))

        gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(InstantCommand(pinch::close, pinch))

        Trigger{gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0 || gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0}
            .whileActiveContinuous(InstantCommand({lift.lift.power = (gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))/2.0}, lift)   )
            .whenInactive(InstantCommand({lift.lift.power = 0.0}, lift))

        schedule(GamepadDrive(drive, { gamepad1.leftY }, { gamepad1.leftX }, { gamepad1.rightX }))

        register(lift, drive)
    }
}