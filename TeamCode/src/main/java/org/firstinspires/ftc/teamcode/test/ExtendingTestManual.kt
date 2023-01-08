package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Intake

@TeleOp
//@Disabled
class ExtendingTestManual: LinearOpMode() {
    lateinit var intake: Intake
    override fun runOpMode() {
        intake = Intake(hardwareMap, telemetry)
        val time = ElapsedTime()
        waitForStart()
        time.reset()
        while(opModeIsActive()) {
            if(gamepad1.right_bumper) {
                intake.extend(Pair(Intake.LEFT_OUT_MAX, Intake.RIGHT_OUT_MAX))
            }
            if(gamepad1.left_bumper) {
                intake.extend(Pair(Intake.LEFT_IN_MIN, Intake.RIGHT_IN_MIN))
            }
            telemetry.update()
        }
    }

}