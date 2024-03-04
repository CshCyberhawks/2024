package frc.robot.commands.cannon

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer

class AutoAmp : Command() {
    override fun initialize() {
        RobotContainer.cannonSystem.ampSpit()
        RobotContainer.cannonSystem.killShooter()
//        RobotContainer.stateMachine.tr
    }

    override fun execute() {
    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
    }
}