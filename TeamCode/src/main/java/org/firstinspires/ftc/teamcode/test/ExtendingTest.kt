package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Intake

@TeleOp
//@Disabled
class ExtendingTest: LinearOpMode() {
    lateinit var intake: Intake
    override fun runOpMode() {
        intake = Intake(hardwareMap, telemetry)
        val time = ElapsedTime()
        waitForStart()
        time.reset()
        while(opModeIsActive()) {
            intake.extend(Pair(Intake.LEFT_OUT_MAX, Intake.RIGHT_OUT_MAX))
            telemetry.addData("Time: ", time.milliseconds())
            telemetry.update()
        }
    }

}