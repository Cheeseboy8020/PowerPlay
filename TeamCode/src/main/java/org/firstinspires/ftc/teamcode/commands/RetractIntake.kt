package org.firstinspires.ftc.teamcode.commands

import android.text.InputFilter.LengthFilter
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension.Companion.LEFT_IN
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension.Companion.RIGHT_IN
import kotlin.math.abs

class RetractIntake(private var intake: IntakeExtension, var delay: Int = 0, var retractPos: Pair<Double, Double> = Pair(LEFT_IN, RIGHT_IN)) : CommandBase() {
    val time = ElapsedTime()
    private var originPos = 0.0
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        time.reset()
        while(time.milliseconds()<delay){}
        originPos = intake.extLeft.position
        time.reset()
        intake.extend(retractPos)
    }


    override fun isFinished(): Boolean {
        return time.milliseconds() >= abs(
            intake.extLeft.position - originPos) /IntakeExtension.EXT_SPEED + 350
    }
}