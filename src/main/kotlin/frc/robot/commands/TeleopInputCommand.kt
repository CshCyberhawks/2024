package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.cannon.AutoShootCommand

class TeleopInputCommand : Command() {

    val prevShoot: Boolean = false
    val prevIntake: Boolean = false
    val prevSpit: Boolean = false
    val prevAmp: Boolean = false
    val prevStow: Boolean = false

    val shootCommand: AutoShootCommand = AutoShootCommand()

    override fun initialize() {
        SmartDashboard.putBoolean("Shoot?", false)
        SmartDashboard.putBoolean("Intake?", false)
        SmartDashboard.putBoolean("Spit?", false)
        SmartDashboard.putBoolean("Amp?", false)
        SmartDashboard.putBoolean("Stow?", false)
    }

    override fun execute() {
        val currentShoot = SmartDashboard.getBoolean("Shoot?", false)
        val currentIntake = SmartDashboard.getBoolean("Intake?", false)
        val currentSpit = SmartDashboard.getBoolean("Spit?", false)
        val currentAmp = SmartDashboard.getBoolean("Amp?", false)
        val currentStow = SmartDashboard.getBoolean("Stow?", false)

        if (currentShoot != prevShoot) {

        } else if (currentIntake != prevIntake) {

        } else if (currentSpit != prevSpit) {

        } else if (currentAmp != prevAmp) {

        } else if (currentStow != prevStow) {

        }
    }
}