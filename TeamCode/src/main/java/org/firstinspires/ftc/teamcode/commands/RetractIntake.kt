package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension.Companion.LEFT_IN
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension.Companion.RIGHT_IN
import kotlin.math.abs

class RetractIntake(private var intake: IntakeExtension, var delay: Int = 0, var retractPos: Pair<Double, Double> = Pair(LEFT_IN, RIGHT_IN)) : CommandBase() {
    val time = ElapsedTime()
    private var originPos = 0.0
    private var originTicks = 0.0
    private var goalTicks = 0.0
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        time.reset()
        while(time.milliseconds()<delay){}
        originPos = intake.extLeft.position
        originTicks = intake.encoder.currentPosition.toDouble()
        goalTicks = ((355*8912)/360.0)*(abs(originPos- LEFT_IN))
        time.reset()
        intake.retract()
    }


    override fun isFinished(): Boolean {
        return abs(originTicks-intake.encoder.currentPosition)<25
    }
}