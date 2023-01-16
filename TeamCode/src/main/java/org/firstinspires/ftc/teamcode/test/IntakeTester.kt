package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.subsystems.LiftArm
import org.firstinspires.ftc.teamcode.util.OpModeType

@TeleOp
//@Disabled
class IntakeTester: LinearOpMode() {
    lateinit var intake: IntakeExtension
    lateinit var lift: LiftArm
    override fun runOpMode() {
        intake = IntakeExtension(hardwareMap, telemetry)
        lift = LiftArm(hardwareMap, LiftArm.Positions.IN_ROBOT, OpModeType.AUTO)
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
                intake.armOut(1)
            }
            if(gamepad1.a){
                intake.armOut(2)
            }
            if(gamepad1.x){
                intake.armOut(3)
            }
            if (gamepad1.b){
                intake.armOut(4)
            }
            if (gamepad1.y){
                intake.armOut(5)
            }
            if(gamepad1.left_stick_button){
                intake.retract()
            }
            if(gamepad1.right_stick_button)
                intake.extend(Pair(IntakeExtension.LEFT_OUT_MAX, IntakeExtension.RIGHT_OUT_MAX))
            if(gamepad2.right_bumper){
                lift.close()
            }
            if(gamepad2.left_bumper){
                lift.open()
            }
            if(gamepad2.y){
                lift.armOut()
            }
            if(gamepad2.a){
                lift.armIn()
            }
        }
    }

}