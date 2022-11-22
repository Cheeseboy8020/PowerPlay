package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Pinch

class PickCone(private val pinch: Pinch) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(pinch)
    }

    override fun initialize() {
        time.reset()
        pinch.close()
    }


    override fun isFinished(): Boolean {
        return time.milliseconds() > 1500
    }
}