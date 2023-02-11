package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoControllerEx
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.subsystems.LiftArm
import org.firstinspires.ftc.teamcode.subsystems.LiftTurret
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.util.OpModeType

@TeleOp
//@Disabled
class ResetExtension: LinearOpMode() {
    lateinit var extRight: Servo
    lateinit var extLeft: Servo
    override fun runOpMode() {
        extRight = hardwareMap.get(Servo::class.java, "extRight")
        extLeft = hardwareMap.get(Servo::class.java, "extRight")
        val controller = extRight.controller as ServoControllerEx
        waitForStart()
        while (opModeIsActive()) {
            extRight.position = 0.5
            extLeft.position = 0.5
            telemetry.addData("pwm range lower right", controller.getServoPwmRange(extRight.portNumber).usPulseLower)
            telemetry.addData("pwm range upper right", controller.getServoPwmRange(extRight.portNumber).usPulseUpper)
            telemetry.addData("pwm range frame right", controller.getServoPwmRange(extRight.portNumber).usFrame)
            telemetry.addData("pwm status right", extRight.controller.pwmStatus)
            telemetry.addData("pwm range lower left", controller.getServoPwmRange(extLeft.portNumber).usPulseLower)
            telemetry.addData("pwm range upper left", controller.getServoPwmRange(extLeft.portNumber).usPulseUpper)
            telemetry.addData("pwm range frame left", controller.getServoPwmRange(extLeft.portNumber).usFrame)
            telemetry.addData("pwm status left", extLeft.controller.pwmStatus)
            telemetry.update()
    }
    }

}