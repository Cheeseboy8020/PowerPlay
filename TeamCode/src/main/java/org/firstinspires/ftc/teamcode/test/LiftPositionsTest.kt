package org.firstinspires.ftc.teamcode.test

import android.util.Log
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.*


@TeleOp(name = "Lift Positions Test")
class LiftPositionsTest: CommandOpMode() {
    override fun initialize() {

        val gamepad1 = GamepadEx(gamepad1)


        val lift = Lift(hardwareMap, telemetry, Lift.Positions.IN_ROBOT)
        val pinch = Pinch(hardwareMap, telemetry)
        Log.w("TeleOp", "Initialized Lift")


        gamepad1.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(GoToPos(lift, Lift.Positions.LOW))

        gamepad1.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(GoToPos(lift, Lift.Positions.IN_ROBOT))

        gamepad1.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(GoToPos(lift, Lift.Positions.HIGH))

        gamepad1.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(GoToPos(lift, Lift.Positions.MEDIUM))

        gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(InstantCommand(pinch::open, pinch))

        gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(InstantCommand(pinch::close, pinch))

        register(lift)
    }
}