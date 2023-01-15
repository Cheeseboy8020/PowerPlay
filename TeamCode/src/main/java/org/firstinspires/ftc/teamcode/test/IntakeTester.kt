package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.util.OpModeType

@TeleOp
//@Disabled
class IntakeTester: LinearOpMode() {
    lateinit var intake: Intake
    override fun runOpMode() {
        intake = Intake(hardwareMap, telemetry)
        waitForStart()
        while(opModeIsActive()) {
            if(gamepad1.right_bumper){
                intake.leftPinch.position=0.5
                intake.rightPinch.position=0.5
            }
            if(gamepad1.left_bumper){
                intake.leftPinch.position = 0.3
                intake.rightPinch.position=0.7
            }
            if(gamepad1.dpad_up){
                intake.arm.position=0.45//reduce to go down
            }
            if(gamepad1.dpad_down){
                intake.arm.position = 0.54 //1 stack
            }
            if(gamepad1.a){
                intake.arm.position = 0.5366//2 stack
            }
            if(gamepad1.x){
                intake.arm.position = 0.5333 //3 stack
            }
            if (gamepad1.b){
                intake.arm.position = 0.53 //4 stack
            }
            if (gamepad1.y){
                intake.arm.position = 0.52 //5 stack
            }
            if(gamepad1.left_stick_button){
                intake.retract()
            }
            if(gamepad1.right_stick_button)
                intake.extend(Pair(Intake.LEFT_OUT_MAX, Intake.RIGHT_OUT_MAX))
        }
    }

}