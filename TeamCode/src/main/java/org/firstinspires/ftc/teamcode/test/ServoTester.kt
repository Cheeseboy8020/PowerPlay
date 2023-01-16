package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.subsystems.LiftArm
import org.firstinspires.ftc.teamcode.util.OpModeType

@TeleOp
//@Disabled
class ServoTester: LinearOpMode() {
    lateinit var intake: IntakeExtension
    lateinit var lift: LiftArm
    lateinit var intakeArm: IntakeArm
    override fun runOpMode() {
        intake = IntakeExtension(hardwareMap, telemetry)
        intakeArm = IntakeArm(hardwareMap, telemetry, OpModeType.AUTO)
        lift = LiftArm(hardwareMap, OpModeType.AUTO)
        waitForStart()
        while(opModeIsActive()) {
            if(gamepad1.right_bumper){
                intakeArm.close()
            }
            if(gamepad1.left_bumper){
                intakeArm.open()
            }
            if(gamepad1.dpad_up){
                intakeArm.armIn()
            }
            if(gamepad1.dpad_down){
                intakeArm.armOut(1)
            }
            if(gamepad1.a){
                intakeArm.armOut(2)
            }
            if(gamepad1.x){
                intakeArm.armOut(3)
            }
            if (gamepad1.b){
                intakeArm.armOut(4)
            }
            if (gamepad1.y){
                intakeArm.armOut(5)
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