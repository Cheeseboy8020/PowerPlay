package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.LiftArm
import org.firstinspires.ftc.teamcode.util.OpModeType

@TeleOp
//@Disabled
class ArmTest: LinearOpMode() {
    lateinit var lift: LiftArm
    override fun runOpMode() {
        lift = LiftArm(hardwareMap, LiftArm.Positions.IN_ROBOT, OpModeType.AUTO)
        waitForStart()
        while(opModeIsActive()) {
            if(gamepad1.right_bumper){
                lift.close()
            }
            if(gamepad1.left_bumper){
                lift.open()
            }
            if(gamepad1.y){
                lift.armOut()
            }
            if(gamepad1.a){
                lift.armIn()
            }
        }
    }

}