package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
//@Disabled
class ExtendingTest: LinearOpMode() {
    lateinit var servo: Servo
    lateinit var servo2: Servo
    override fun runOpMode() {
        servo = hardwareMap.get(Servo::class.java, "rightPinch")//left irl
        servo2 = hardwareMap.get(Servo::class.java, "leftPinch")//right irl
        waitForStart()
        while(opModeIsActive()) {
            telemetry.addData("Servo Direction1", servo.direction)
            telemetry.addData("Servo Direction2", servo2.direction)
            telemetry.update()
            if(gamepad1.right_bumper){
                servo.position=1.0
                servo2.position=0.0
            }
            if(gamepad1.left_bumper){
                servo.position = 0.3
                servo2.position=0.7
            }
        }
    }

}