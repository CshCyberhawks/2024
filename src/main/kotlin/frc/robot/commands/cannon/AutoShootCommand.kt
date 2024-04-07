package frc.robot.commands.cannon

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.util.Timer

class AutoShootCommand : Command() {
    val timer = Timer()

    override fun initialize() {
        RobotContainer.cannonSystem.shoot()
        timer.reset()
    }

    override fun execute() {
        if (RobotContainer.stateMachine.shooterReady && !timer.isRunning) {
            timer.start()
        }

        if (timer.hasElapsed(.15)) {
            RobotContainer.cannonSystem.feed()
        }

        SmartDashboard.putBoolean("shooter ready", RobotContainer.stateMachine.shooterReady)
    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.cannonSystem.killIntake()
    }
}
