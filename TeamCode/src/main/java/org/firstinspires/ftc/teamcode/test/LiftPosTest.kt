package org.firstinspires.ftc.teamcode.test

import android.util.Log
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
class LiftPosTest: CommandOpMode() {

    override fun initialize() {

        val gamepad1 = GamepadEx(gamepad1)


        val lift = Lift(hardwareMap, Lift.Positions.IN_ROBOT, OpModeType.AUTO)
        Log.w("TeleOp", "Initialized Lift")
        gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(ExtendLift(lift, Lift.Positions.HIGH))
        gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(ExtendLift(lift, Lift.Positions.IN_ROBOT))
        register(lift)
    }
}