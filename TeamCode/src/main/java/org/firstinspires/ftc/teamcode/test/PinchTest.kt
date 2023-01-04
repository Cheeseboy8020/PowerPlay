package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
//@Disabled
class PinchTest: LinearOpMode() {
    lateinit var leftPinch: Servo
    lateinit var leftPinch2: Servo
    lateinit var rightPinch: Servo
    lateinit var rightPinch2: Servo
    lateinit var intakeArm: Servo
    lateinit var liftArm: Servo
    override fun runOpMode() {
        rightPinch = hardwareMap.get(Servo::class.java, "rightPinch")
        rightPinch2 = hardwareMap.get(Servo::class.java, "rightPinch2")
        leftPinch = hardwareMap.get(Servo::class.java, "leftPinch")
        leftPinch2 = hardwareMap.get(Servo::class.java, "leftPinch2")
        intakeArm = hardwareMap.get(Servo::class.java, "intakeArm")
        liftArm = hardwareMap.get(Servo::class.java, "liftArm")
        waitForStart()
        while(opModeIsActive()) {
            if(gamepad1.left_bumper){
                leftPinch.position=0.12
                rightPinch.position=0.18
            }
            if(gamepad1.right_bumper){
                leftPinch.position = 0.25
                rightPinch.position=0.05
            }
            if(gamepad1.left_stick_button){
                leftPinch2.position = 0.13//reduce to go down
                rightPinch2.position = 0.67
            }
            if(gamepad1.right_stick_button){
                leftPinch2.position = 0.0//reduce to go down
                rightPinch2.position = 0.8
            }
            if(gamepad1.dpad_up){
                intakeArm.position=0.45//reduce to go down
            }
            if(gamepad1.dpad_down){
                intakeArm.position = 0.55 //1 stack
            }
            if(gamepad1.left_trigger>0.0){
                liftArm.position = 0.2 //reduce to go down
            }
            if(gamepad1.right_trigger>0.0){
                 liftArm.position = 0.06//1 stack
            }
            if(gamepad1.a){
                intakeArm.position = 0.545 //2 stack
            }
            if(gamepad1.x){
                intakeArm.position = 0.54 //3 stack
            }
            if (gamepad1.b){
                intakeArm.position = 0.53 //4 stack
            }
            if (gamepad1.y){
                intakeArm.position = 0.52 //5 stack
            }
            if (gamepad1.dpad_right){
                leftPinch.position=0.12
                rightPinch.position=0.18
                intakeArm.position = 0.55 //1 stack
                sleep(1000)
                liftArm.position = 0.06//1 stack
                leftPinch2.position = 0.13//reduce to go down
                rightPinch2.position = 0.67
                leftPinch.position = 0.25
                rightPinch.position=0.05
                sleep(1000)
                intakeArm.position=0.45//reduce to go down
                sleep(1000)
                leftPinch2.position = 0.0//reduce to go down
                rightPinch2.position = 0.8
                leftPinch.position=0.12
                rightPinch.position=0.18
                sleep(1000)
                intakeArm.position = 0.55 //1 stack
                liftArm.position = 0.2
            }
        }
    }

}