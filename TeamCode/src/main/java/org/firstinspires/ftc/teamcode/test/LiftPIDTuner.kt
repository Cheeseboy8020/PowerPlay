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
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.commands.LiftGoToPos
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.util.OpModeType

@Config
@Disabled
@Autonomous
class LiftPIDTuner : CommandOpMode() {


    private lateinit var lift: Lift

    private var target = 0.0


    override fun initialize() {

        lift = Lift(hardwareMap, telemetry, Lift.Positions.IN_ROBOT, OpModeType.AUTO)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)



        val driver = GamepadEx(gamepad1)

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                SequentialCommandGroup(
                    InstantCommand({target = Lift.Positions.HIGH.targetPosition.toDouble() }),
                    LiftGoToPos(lift, Lift.Positions.HIGH),
                    WaitCommand(1000),
                    InstantCommand({target = Lift.Positions.IN_ROBOT.targetPosition.toDouble()}),
                    LiftGoToPos(lift, Lift.Positions.IN_ROBOT)
                )
            )
    }

    override fun run() {
        super.run()

        telemetry.addData("Target Velo", lift.targetVelo)
        telemetry.addData("Actual Velo", lift.lift.velocity)

        telemetry.addData("Target Pos", target)
        telemetry.addData("Actual Pos", lift.lift.currentPosition)

        telemetry.update()
    }
}