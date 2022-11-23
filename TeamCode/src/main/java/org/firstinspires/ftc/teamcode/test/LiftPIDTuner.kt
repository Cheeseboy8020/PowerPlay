package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.LiftGoToPos
import org.firstinspires.ftc.teamcode.subsystems.Lift

@Config
@Autonomous
class LiftPIDTuner : CommandOpMode() {


    private lateinit var lift: Lift

    private var target = 0.0


    override fun initialize() {

        lift = Lift(hardwareMap, telemetry, Lift.Positions.IN_ROBOT)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)



        val driver = GamepadEx(gamepad1)

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                SequentialCommandGroup(
                    InstantCommand({target = 3000.0}),
                    LiftGoToPos(lift, Lift.Positions.HIGH),
                    WaitCommand(1000),
                    InstantCommand({target = 0.0}),
                    LiftGoToPos(lift, Lift.Positions.IN_ROBOT)
                )
            )
    }

    override fun run() {
        super.run()

        telemetry.addData("Target", target)
        telemetry.addData("Actual", lift.lift.currentPosition)

        telemetry.update()
    }
}