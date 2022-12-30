package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
//@Disabled
class PinchTest: LinearOpMode() {
    lateinit var servo: Servo
    lateinit var servo2: Servo
    override fun runOpMode() {
        servo = hardwareMap.get(Servo::class.java, "rightPinch")
        servo2 = hardwareMap.get(Servo::class.java, "leftPinch")
        waitForStart()
        while(opModeIsActive()) {
            if(gamepad1.right_bumper){
                servo.position=0.25
                servo2.position=0.2
            }
            if(gamepad1.left_bumper){
                servo.position = 0.3
                servo2.position=0.0
            }
        }
    }

}