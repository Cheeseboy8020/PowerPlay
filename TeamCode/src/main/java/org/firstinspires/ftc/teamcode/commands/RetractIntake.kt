package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Intake
import kotlin.math.abs

class RetractIntake(private var intake: Intake) : CommandBase() {
    val time = ElapsedTime()
    private var originPos = 0.0
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        originPos = intake.extLeft.position
        time.reset()
        intake.retract()
    }


    override fun isFinished(): Boolean {
        return time.milliseconds() >= abs(
            intake.extLeft.position - originPos) /Intake.EXT_SPEED + 100
    }
}