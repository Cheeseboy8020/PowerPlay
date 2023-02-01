package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.util.OpModeType

@TeleOp
//@Disabled
class ExtendingTest: LinearOpMode() {
    lateinit var intake: IntakeExtension
    override fun runOpMode() {
        intake = IntakeExtension(hardwareMap, telemetry, OpModeType.AUTO)
        val time = ElapsedTime()
        waitForStart()
        time.reset()
        while(opModeIsActive()) {
            telemetry.addData("Time: ", time.milliseconds())
            telemetry.update()
        }
    }

}