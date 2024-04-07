package frc.robot.commands.cannon

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.util.Timer

class HalfSpitCannon() : Command() {
    val timer = Timer()

    override fun initialize() {
        RobotContainer.cannonSystem.spit()
        timer.reset()
    }

    override fun execute() {
        if (RobotContainer.stateMachine.noteState == NoteState.Intaking && !timer.isRunning) {
            timer.start()
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(.02)
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
        timer.reset()
    }
}
