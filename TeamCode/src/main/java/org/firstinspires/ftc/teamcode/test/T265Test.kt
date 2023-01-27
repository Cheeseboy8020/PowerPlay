package org.firstinspires.ftc.teamcode.test

import com.arcrobotics.ftclib.geometry.Transform2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.spartronics4915.lib.T265Camera
import org.firstinspires.ftc.teamcode.cv.SignalScanner

@Autonomous
@Disabled
class T265Test: LinearOpMode() {
    lateinit var slamera: T265Camera
    override fun runOpMode() {
        slamera = T265Camera(Transform2d(),
            0.8, hardwareMap.appContext)
    }

}