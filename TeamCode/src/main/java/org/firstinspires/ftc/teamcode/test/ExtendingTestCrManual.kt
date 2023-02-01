package org.firstinspires.ftc.teamcode.test

import com.alphago.agDistanceLocalization.filters.MedianFilter
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.util.Encoder
import org.firstinspires.ftc.teamcode.util.OpModeType

@TeleOp
//@Disabled
class ExtendingTestCrManual: LinearOpMode() {
    lateinit var intake: IntakeExtension
    override fun runOpMode() {
        intake = IntakeExtension(hardwareMap, telemetry, OpModeType.AUTO)
        var motor = hardwareMap.get(DcMotorEx::class.java, "encoder")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        var encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "encoder"))
        encoder.direction = Encoder.Direction.REVERSE
        val time = ElapsedTime()
        waitForStart()
        time.reset()
        while (opModeIsActive()){
            intake.extRight.power = -gamepad1.right_stick_y.toDouble()
            intake.extLeft.power = -gamepad1.right_stick_y.toDouble()
            telemetry.addData("pos", encoder.currentPosition)
            telemetry.update()
        }
    }

}