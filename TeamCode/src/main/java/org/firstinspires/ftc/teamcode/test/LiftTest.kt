package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.util.OpModeType

@TeleOp
//@Disabled
class LiftTest: LinearOpMode() {
    lateinit var lift: Lift
    override fun runOpMode() {
        lift = Lift(hardwareMap, Lift.Positions.IN_ROBOT, OpModeType.AUTO)
        waitForStart()
        while(opModeIsActive()) {
            lift.setPower((gamepad1.right_trigger - gamepad1.left_trigger).toDouble())

            // Left Bumper is being pressed, so send left and right "trigger" values to left and right rumble motors.
            gamepad1.rumble(
                gamepad1.left_trigger.toDouble(),
                gamepad1.right_trigger.toDouble(),
                Gamepad.RUMBLE_DURATION_CONTINUOUS
            )
            telemetry.addData("lift", lift.currentPosition)
            telemetry.update()
        }
    }

}