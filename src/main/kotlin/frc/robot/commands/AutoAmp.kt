package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.IntakeState
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.GoToPoseTrunk
import frc.robot.util.Timer

class AutoAmp : Command() {
    val timer = Timer()

    override fun initialize() {
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseTrunk(TrunkPose.AMP_GOING).andThen(GoToPoseAndHoldTrunk(TrunkPose.AMP))
    }

    override fun execute() {
//        if (RobotContainer.xboxController.start().asBoolean) {
//            RobotContainer.cannonSystem.ampSpit()
//        } else {
//            RobotContainer.cannonSystem.killIntake()
//        }
        if (RobotContainer.actuallyDoAmp && !timer.isRunning) {
            timer.start()
            RobotContainer.cannonSystem.ampSpit()
        }
    }

    override fun isFinished(): Boolean {
//        return RobotContainer.stateMachine.noteState == NoteState.Empty && RobotContainer.stateMachine.intakeState != IntakeState.AmpSpitting
        return timer.hasElapsed(0.5)
    }

    override fun end(interrupted: Boolean) {
        timer.reset()
        RobotContainer.cannonSystem.killIntake()
        RobotContainer.actuallyDoAmp = false
        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseTrunk(TrunkPose.AMP_LEAVING).andThen(GoToPoseAndHoldTrunk(TrunkPose.STOW))
    }
}