package frc.robot.commands.cannon

import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.util.Timer

class ShooterFeedCannon() : Command() {

    val intakeTimer = Timer()
    override fun initialize() {
        RobotContainer.cannonSystem.shooterFeed()
        intakeTimer.reset()
    }

    override fun execute() {
        if (RobotContainer.stateMachine.noteState == NoteState.Stored && !intakeTimer.isRunning) {
            intakeTimer.start()
        }
    }

    override fun isFinished(): Boolean {
        return intakeTimer.hasElapsed(0.1)
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
        RobotContainer.cannonSystem.killShooter()
    }
}