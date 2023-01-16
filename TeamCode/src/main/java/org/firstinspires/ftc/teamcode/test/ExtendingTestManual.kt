package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

@TeleOp
//@Disabled
class ExtendingTestManual: LinearOpMode() {
    lateinit var intake: IntakeExtension
    override fun runOpMode() {
        intake = IntakeExtension(hardwareMap, telemetry)
        val time = ElapsedTime()
        waitForStart()
        time.reset()
        while(opModeIsActive()) {
            if(gamepad1.right_bumper) {
                intake.extend(Pair(IntakeExtension.LEFT_OUT_MAX, IntakeExtension.RIGHT_OUT_MAX))
            }
            if(gamepad1.left_bumper) {
                intake.extend(Pair(IntakeExtension.LEFT_IN_MIN, IntakeExtension.RIGHT_IN_MIN))
            }
            telemetry.update()
        }
    }

}