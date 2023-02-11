package org.firstinspires.ftc.teamcode.commands
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.LiftTurret
import kotlin.math.abs

class LiftTurretPosition(private var turret: LiftTurret, private var pos : Double) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(turret)
    }

    override fun initialize() {
        time.reset()
        turret.pos = pos
    }

    override fun isFinished(): Boolean {
        return abs(turret.pos - pos) < 0.1
    }
}