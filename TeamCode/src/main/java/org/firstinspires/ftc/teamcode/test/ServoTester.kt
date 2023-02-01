package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.subsystems.LiftArm
import org.firstinspires.ftc.teamcode.subsystems.LiftTurret
import org.firstinspires.ftc.teamcode.util.OpModeType

@TeleOp
//@Disabled
class ServoTester: LinearOpMode() {
    lateinit var intake: IntakeExtension
    lateinit var lift: LiftArm
    lateinit var turret: LiftTurret
    lateinit var intakeArm: IntakeArm
    override fun runOpMode() {
        intake = IntakeExtension(hardwareMap, telemetry, OpModeType.AUTO)
        intakeArm = IntakeArm(hardwareMap, telemetry, OpModeType.AUTO)
        lift = LiftArm(hardwareMap, OpModeType.TELEOP)
        turret = LiftTurret(hardwareMap, OpModeType.TELEOP)
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
            if(gamepad2.y){
                lift.armOut()
            }
            if(gamepad2.a){
                lift.armIn()
            }
            if(gamepad2.dpad_down){
                turret.setAngle(0.0)
            }
            if (gamepad2.dpad_up){
                turret.setAngle(180.0)
            }
            if (gamepad2.dpad_left){
                turret.setAngle(90.0)
            }
            if (gamepad2.dpad_right){
                turret.setAngle(270.0)
            }
            /*if(gamepad2.right_trigger.toDouble()!=0.0){
                turret.turret.position = gamepad2.right_trigger.toDouble()
            }*/
            intake.extRight.power = -gamepad1.right_stick_y.toDouble()
            intake.extLeft.power = -gamepad1.right_stick_y.toDouble()
            telemetry.addData("turretPos", turret.getPos())
            telemetry.addData("turretPosProgram", turret.turret.position)
            telemetry.update()
        }
    }

}