package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.trunk.CoastAngleMovePosition
import frc.robot.commands.trunk.GoToPoseTrunk

class AutoIntakeAndShoot : Command() {

    var hasIntake: Boolean = false
    var hasAlmostSpit: Boolean = false

    override fun initialize() {
        RobotContainer.cannonSystem.closeShoot()
        RobotContainer.cannonSystem.intake()
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
//            RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)
//            RobotContainer.stateMachine.currentTrunkCommand = CoastAngleMovePosition(TrunkPose.INTAKE_PREP).andThen(GoToPoseAndHoldTrunk(TrunkPose.STOW))
//            RobotContainer.cannonSystem.killIntake()
//            intakeCommand.cancel()
//            intakeCommand = HalfSpitCannon().andThen(IntakeCannon())
//            intakeCommand.schedule()
            hasIntake = true
            println("spitting back out")
        }
        if (RobotContainer.stateMachine.noteState == NoteState.Intaking && hasIntake && !hasAlmostSpit) {
            hasAlmostSpit = true
            println("intaking back up")
        }

        if (hasAlmostSpit) {
            RobotContainer.cannonSystem.noteEntryTime = -1.0
        }
    }

    override fun isFinished(): Boolean =
        false//RobotContainer.stateMachine.noteState == NoteState.Stored //&& intakeCommand.isFinished // && hasIntake && hasAlmostSpit

    override fun end(interrupted: Boolean) {
        // intakeCommand.cancel()
//        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseTrunk(TrunkPose.STOW)
    }
}
