package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
//@Disabled
class ArmTest: LinearOpMode() {
    lateinit var servo: Servo
    override fun runOpMode() {
        servo = hardwareMap.get(Servo::class.java, "intakeArm")
        waitForStart()
        while(opModeIsActive()) {
            if(gamepad1.right_bumper){
                servo.position=0.32//reduce to go down
            }
            if(gamepad1.left_bumper){
                servo.position = 0.41 //reduce to go up
            }
        }
    }

}