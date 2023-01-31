package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.alphago.agDistanceLocalization.filters.LowPassFilter
import com.alphago.agDistanceLocalization.filters.MedianFilter
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

@TeleOp
@Config
//@Disabled
class ExtendingTestCr: LinearOpMode() {
    companion object{
        @JvmField var THRESHOLD = 0.4
        @JvmField var SPEED = 1.0
    }
    lateinit var intake: IntakeExtension
    override fun runOpMode() {
        intake = IntakeExtension(hardwareMap, telemetry)
        lateinit var analog: AnalogInput
        analog = hardwareMap.get(AnalogInput::class.java, "dist")
        var lp = LowPassFilter(0.5)
        waitForStart()
        while (opModeIsActive()){
            while (lp.estimate(analog.voltage)>THRESHOLD){
                intake.extRight.power = SPEED
                intake.extLeft.power = SPEED
                telemetry.addData("DistLowPass: ", lp.estimate(analog.voltage))
                telemetry.update()
            }
            intake.extRight.power = 0.0
            intake.extLeft.power = 0.0
            telemetry.addData("DistLowPass: ", lp.estimate(analog.voltage))
            telemetry.update()
        }
    }
}