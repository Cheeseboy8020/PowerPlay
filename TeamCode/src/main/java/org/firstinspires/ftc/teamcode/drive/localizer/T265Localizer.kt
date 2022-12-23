package org.firstinspires.ftc.teamcode.drive.localizer

import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.spartronics4915.lib.T265Camera
import com.spartronics4915.lib.T265Camera.CameraUpdate
import com.spartronics4915.lib.T265Camera.PoseConfidence


class T265Localizer(drive: MecanumDrive, hardwareMap: HardwareMap): Localizer {

    var up: CameraUpdate

    companion object{
        @JvmField var xOffset = 0.0 // Inches
        @JvmField var yOffset = 0.0 // Inches
        @JvmField var hOffset = 0.0 // Degrees
        @JvmField var slamra: T265Camera? = null
    }

    val encoderLoc: MecanumDrive.MecanumLocalizer

    override var poseEstimate: Pose2d
        get() = Pose2d(up.pose.x/0.0254, up.pose.y/0.0254, up.pose.rotation.radians)
        set(value) {slamra!!.setPose(com.arcrobotics.ftclib.geometry.Pose2d(value.x*0.0254, value.y*0.0254, Rotation2d(value.heading)))
        encoderLoc.poseEstimate = value}

    override val poseVelocity: Pose2d
        get() = Pose2d(up.velocity.vxMetersPerSecond/0.0254, up.velocity.vyMetersPerSecond/0.0254, up.velocity.omegaRadiansPerSecond)

    var poseConfidence: PoseConfidence
        get() = up.confidence


    override fun update() {
        encoderLoc.update()
        slamra!!.sendOdometry(encoderLoc.poseVelocity!!.x*0.0254, encoderLoc.poseVelocity!!.y*0.0254)
        up = slamra!!.lastReceivedCameraUpdate
    }

    fun setOdometryInfo(){
        //slamra!!.
    }

    init {
        slamra = T265Camera(Transform2d(Translation2d(xOffset*0.0254, yOffset*0.0254), Rotation2d(Math.toRadians(
            hOffset))), 0.1, hardwareMap.appContext)
        encoderLoc = MecanumDrive.MecanumLocalizer(drive)
        slamra!!.start()
        up = slamra!!.lastReceivedCameraUpdate
        poseConfidence = up.confidence
    }
}