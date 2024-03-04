package frc.robot.commands.cannon

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer

class AutoShootCommand : Command() {
    override fun initialize() {
        RobotContainer.cannonSystem.shoot()
    }

    override fun execute() {
        if (RobotContainer.stateMachine.shooterReady) {
            RobotContainer.cannonSystem.feed()
        }
    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.cannonSystem.killIntake()
    }
}
