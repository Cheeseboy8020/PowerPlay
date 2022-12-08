package org.firstinspires.ftc.teamcode.test

import android.util.Log
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.util.OpModeType


@TeleOp(name = "Lift Positions Test")
@Disabled
class LiftPositionsTest: CommandOpMode() {
    override fun initialize() {

        val gamepad1 = GamepadEx(gamepad1)


        val lift = Lift(hardwareMap, telemetry, Lift.Positions.IN_ROBOT, OpModeType.AUTO)
        val pinch = Pinch(hardwareMap, telemetry)
        Log.w("TeleOp", "Initialized Lift")


        gamepad1.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.LOW))

        gamepad1.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.IN_ROBOT))

        gamepad1.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.HIGH))

        gamepad1.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.MEDIUM))

        gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(InstantCommand(pinch::open, pinch))

        gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(InstantCommand(pinch::close, pinch))

        register(lift)
    }
}