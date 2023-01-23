package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
//@Disabled
class ServoReset: LinearOpMode() {
    lateinit var servo: Servo
    override fun runOpMode() {
        servo = hardwareMap.get(Servo::class.java, "extLeft")
        waitForStart()
        while(opModeIsActive()) {
            servo.position = 0.4
        }
    }

}