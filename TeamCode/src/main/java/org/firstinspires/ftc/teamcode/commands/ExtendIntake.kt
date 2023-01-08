package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Intake
import kotlin.math.abs

class ExtendIntake(private var intake: Intake, private var driveVec: Vector2d, private var goalVec: Vector2d) : CommandBase() {
    val time = ElapsedTime()
    private var originPos = 0.0
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        originPos = intake.extLeft.position
        time.reset()
        intake.extend(Intake.calcPos(driveVec, goalVec))
    }


    override fun isFinished(): Boolean {
        return time.milliseconds() >= abs(
            intake.extLeft.position - originPos) /Intake.EXT_SPEED + 100
    }
}