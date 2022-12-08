package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad

@TeleOp
@Disabled
class LiftTest: LinearOpMode() {
    lateinit var lift: DcMotorEx
    override fun runOpMode() {
        lift = hardwareMap.get(DcMotorEx::class.java, "lift")
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        waitForStart()
        while(opModeIsActive()) {
            lift.power = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()

            // Left Bumper is being pressed, so send left and right "trigger" values to left and right rumble motors.
            gamepad1.rumble(
                gamepad1.left_trigger.toDouble(),
                gamepad1.right_trigger.toDouble(),
                Gamepad.RUMBLE_DURATION_CONTINUOUS
            )
        }
    }

}