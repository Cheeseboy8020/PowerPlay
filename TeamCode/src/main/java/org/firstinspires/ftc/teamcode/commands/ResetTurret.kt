package org.firstinspires.ftc.teamcode.commands
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.LiftTurret
import org.firstinspires.ftc.teamcode.subsystems.LiftTurret.Companion.TURRET_RESET
import kotlin.math.abs

class ResetTurret(private var turret: LiftTurret) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(turret)
    }

    override fun initialize() {
        time.reset()
        turret.pos = TURRET_RESET
    }

    override fun isFinished(): Boolean {
        return abs(turret.pos - 0.3) < 0.1
    }
}