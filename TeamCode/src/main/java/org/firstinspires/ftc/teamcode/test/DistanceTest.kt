package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.alphago.agDistanceLocalization.filters.LowPassFilter
import com.alphago.agDistanceLocalization.filters.MedianFilter
import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name = "DistTest")
class DistanceTest: LinearOpMode() {
    lateinit var analog: AnalogInput
    override fun runOpMode() {
        var dashboard = FtcDashboard.getInstance();
        telemetry = MultipleTelemetry(this.telemetry, dashboard.telemetry)
        analog = hardwareMap.get(AnalogInput::class.java, "dist")
        var mf = MedianFilter(3)
        var lp = LowPassFilter(0.5)
        waitForStart()
        while(opModeIsActive()&&!isStopRequested) {
            mf.push(analog.voltage)
            telemetry.addData("DistLowPass: ", lp.estimate(analog.voltage))
            telemetry.addData("DistMedian: ", mf.median())
            telemetry.update()
        }
    }

}