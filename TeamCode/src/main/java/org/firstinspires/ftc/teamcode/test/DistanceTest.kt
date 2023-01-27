package org.firstinspires.ftc.teamcode.test

import com.alphago.agDistanceLocalization.filters.LowPassFilter
import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name = "DistTest")
class DistanceTest: LinearOpMode() {
    lateinit var analog: AnalogInput
    lateinit var i2c: MB1242Ex
    override fun runOpMode() {
        analog = hardwareMap.get(AnalogInput::class.java, "dist")
        val agLowPass = LowPassFilter(0.99)
        val mbLowPass = LowPassFilter(0.99)
        i2c = hardwareMap.get(DistanceSensor::class.java, "dist2") as MB1242Ex
        waitForStart()
        while(opModeIsActive()&&!isStopRequested) {
            telemetry.addData("DistAg: ", agLowPass.estimate(90.0 * analog.voltage - 12.0))
            telemetry.addData("DistMb: ",  mbLowPass.estimate(((analog.voltage / (3.3/1024)*6) - 300)/25.4))
            telemetry.addData("Disti2c: ", i2c.getDistanceAsync(DistanceUnit.INCH))
            telemetry.update()
        }
    }

}