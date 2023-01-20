package org.firstinspires.ftc.teamcode.commands

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandOpMode
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import java.util.function.DoubleSupplier


class StopT265(
    val opMode: CommandOpMode
) : CommandBase() {
    override fun execute() {
        if(opMode.isStopRequested){
            T265Localizer.slamera!!.stop()
        }
    }
}