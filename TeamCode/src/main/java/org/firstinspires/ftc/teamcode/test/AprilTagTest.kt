package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.cv.SignalScanner

@Autonomous
@Disabled
class AprilTagTest: LinearOpMode() {
    lateinit var cv: SignalScanner
    override fun runOpMode() {
        cv = SignalScanner(hardwareMap, telemetry)
        waitForStart()
        while(opModeIsActive()&&!isStopRequested) {
            telemetry.addData("Detected: ", cv.scanBarcode())
            telemetry.update()
            if(isStopRequested){
                cv.stop()
            }
        }
        cv.stop()
    }

}