package frc.robot.commands.cannon

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPosition

class AutoIntake : Command() {

    var hasIntake: Boolean = false
    var hasAlmostSpit: Boolean = false
    override fun initialize() {
//        RobotContainer.trunkSystem.goToCustom()
        RobotContainer.cannonSystem.intake()
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.INTAKE
        hasIntake = false
        hasAlmostSpit = false
    }

    override fun execute() {
//        if (RobotContainer.stateMachine.noteState == NoteState.Intaking) {
//            RobotContainer.cannonSystem.feed()
//        }

        if (RobotContainer.stateMachine.noteState == NoteState.Stored && !hasIntake) {
//            RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.STOW
            RobotContainer.cannonSystem.killIntake()
            RobotContainer.cannonSystem.spit()
            hasIntake = true
            println("spitting back out")
        }
        if (RobotContainer.stateMachine.noteState == NoteState.Intaking && hasIntake && !hasAlmostSpit) {
            RobotContainer.cannonSystem.intake()
            hasAlmostSpit = true
            println("intaking back up")
        }

        if (hasAlmostSpit) {
            RobotContainer.cannonSystem.noteEntryTime = -1.0
        }

    }

    override fun isFinished(): Boolean =
        RobotContainer.stateMachine.noteState == NoteState.Stored && hasIntake && hasAlmostSpit

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
        RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.STOW
    }
}