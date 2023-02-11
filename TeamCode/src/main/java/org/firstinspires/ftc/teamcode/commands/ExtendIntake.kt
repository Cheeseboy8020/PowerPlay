package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.alphago.agDistanceLocalization.geometry.Pose
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import kotlin.math.abs

@Config
class ExtendIntake(private var intake: IntakeExtension, var extPos: Double) : CommandBase() {
    val time = ElapsedTime()
    private var originPos = 0.0
    private var originTicks = 0.0
    private var goalTicks = 0.0

    init {
        addRequirements(intake)
    }

    override fun initialize() {
        originPos = intake.extLeft.position
        originTicks = intake.encoder.currentPosition.toDouble()
        goalTicks = ((355*8912)/360.0)*(abs(originPos-extPos))
        time.reset()
        intake.extend(Pair(extPos, 1-extPos))
    }


    override fun isFinished(): Boolean {
        return abs(originTicks-intake.encoder.currentPosition)<25
    }
}