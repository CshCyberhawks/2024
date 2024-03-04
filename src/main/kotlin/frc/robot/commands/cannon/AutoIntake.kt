package frc.robot.commands.cannon

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer

class AutoIntake : Command() {
    override fun initialize() {
        RobotContainer.cannonSystem.intake()
        RobotContainer.cannonSystem.killShooter()
//        RobotContainer.stateMachine.trunkState = TrunkState.Floor
    }

    override fun execute() {
//        if (RobotContainer.stateMachine.noteState == NoteState.Intaking) {
//            RobotContainer.cannonSystem.feed()
//        }
    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Stored
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
    }
}