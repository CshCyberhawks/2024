package frc.robot.commands.automatic

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command

class AutoClimbCommand : Command() {
    override fun initialize() {
        super.initialize()
    }

    override fun execute() {
        val pulldown = SmartDashboard.getBoolean("Pulldown Climb?", false)

        if (!pulldown) {

        } else {

        }
    }

    override fun isFinished(): Boolean {
        return super.isFinished()
    }

    override fun end(interrupted: Boolean) {
        SmartDashboard.putBoolean("Pulldown Climb?", false)
    }
}