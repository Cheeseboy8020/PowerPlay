package org.firstinspires.ftc.teamcode.autonomous

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.function.Consumer

/**
 * Command that runs infinitely and will constantly clear the bulk cache each loop.
 */
class BulkCacheCommand(hardwareMap: HardwareMap) : CommandBase() {
    private val allHubs: List<LynxModule>
    override fun initialize() {
        allHubs.forEach(Consumer { hub: LynxModule ->
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        })
    }

    override fun execute() {
        allHubs.forEach(Consumer { obj: LynxModule -> obj.clearBulkCache() })
    }

    init {
        allHubs = hardwareMap.getAll(LynxModule::class.java)
    }
}