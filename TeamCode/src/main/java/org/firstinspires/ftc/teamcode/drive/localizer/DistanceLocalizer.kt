package org.firstinspires.ftc.teamcode.drive.localizer

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.alphago.agDistanceLocalization.ThreeSensorLocalization
import com.alphago.agDistanceLocalization.geometry.Pose
import com.alphago.agDistanceLocalization.roadrunner.asRoadRunnerCoords
import com.alphago.agDistanceLocalization.roadrunner.asUnitCircleHeading
import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive


class DistanceLocalizer(hardwareMap: HardwareMap) : Localizer {
    var analog1: AnalogInput
    var analog2: AnalogInput
    var i2c: MB1242Ex
    val tsl = ThreeSensorLocalization(Pose(0.0, 0.0, 0.0), Pose(0.0, 0.0, 0.0), Pose(0.0, 0.0, 0.0), 30.0)
    lateinit var pose: Pose
    override var poseEstimate: Pose2d
        get() = asRoadRunnerCoords(pose)
        set(value) {
            pose = Pose(value.x+72.0, 72.0+value.y, asUnitCircleHeading(value.heading))
        }
    override var poseVelocity: Pose2d? = null
        private set

    override fun update() {
        pose = tsl.update(90.0*analog1.voltage-12, i2c.getDistance(DistanceUnit.INCH), 90*analog2.voltage-12, pose.rad)
    }

    init {
        analog1 = hardwareMap.get(AnalogInput::class.java, "analog1")
        analog2 = hardwareMap.get(AnalogInput::class.java, "analog2")
        i2c = hardwareMap.get(DistanceSensor::class.java, "i2c") as MB1242Ex
    }
}