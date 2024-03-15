package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.trunk.CoastAngleMovePosition
import frc.robot.commands.trunk.GoToPoseTrunk
import frc.robot.commands.trunk.CoastTrunk
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk

class AutoIntake : Command() {

    var hasIntake: Boolean = false
    var hasAlmostSpit: Boolean = false

    override fun initialize() {
        RobotContainer.cannonSystem.intake()
        RobotContainer.cannonSystem.killShooter()
        hasIntake = false
        hasAlmostSpit = false

        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseTrunk(TrunkPose.INTAKE_PREP)

//        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseTrunk(TrunkPose.INTAKE_PREP).andThen(CoastAngleMovePosition(TrunkPose.INTAKE)).andThen(CoastAngleMovePosition(TrunkPose.INTAKE_PREP).andThen(GoToPoseAndHoldTrunk(TrunkPose.STOW)))

    }

    override fun execute() {


        if (RobotContainer.stateMachine.currentTrunkCommand.isFinished && !hasIntake) {
            RobotContainer.stateMachine.currentTrunkCommand = CoastAngleMovePosition(TrunkPose.INTAKE)
        }

        if (RobotContainer.stateMachine.noteState == NoteState.Stored && !hasIntake) {
            RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)
//            RobotContainer.stateMachine.currentTrunkCommand = CoastAngleMovePosition(TrunkPose.INTAKE_PREP).andThen(GoToPoseAndHoldTrunk(TrunkPose.STOW))
            RobotContainer.cannonSystem.killIntake()
            RobotContainer.cannonSystem.spit()
            hasIntake = true
//            println("spitting back out")
        }
        if (RobotContainer.stateMachine.noteState == NoteState.Intaking && hasIntake && !hasAlmostSpit) {
            RobotContainer.cannonSystem.intake()
            hasAlmostSpit = true
//            println("intaking back up")
        }

        if (hasAlmostSpit) {
            RobotContainer.cannonSystem.noteEntryTime = -1.0
        }
    }

    override fun isFinished(): Boolean = RobotContainer.stateMachine.noteState == NoteState.Stored && hasIntake && hasAlmostSpit

    override fun end(interrupted: Boolean) {
        RobotContainer.cannonSystem.killIntake()
//        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseTrunk(TrunkPose.STOW)
    }
}