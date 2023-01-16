package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import kotlin.math.abs

class RetractIntake(private var intake: IntakeExtension) : CommandBase() {
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
            intake.extLeft.position - originPos) /IntakeExtension.EXT_SPEED + 100
    }
}