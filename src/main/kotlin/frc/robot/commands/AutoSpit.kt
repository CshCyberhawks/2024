package frc.robot.commands.cannon

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.CalibrateTrunk
import frc.robot.TrunkPose
import frc.robot.Robot
import frc.robot.IntakeState
import frc.robot.util.Timer

class AutoSpit : Command() {
    //    val goToSpitPose = GoToPoseAndHoldTrunk(TrunkPose.CalibrationAngle)
    val timer = Timer()

    override fun initialize() {
        RobotContainer.cannonSystem.killShooter()

//        RobotContainer.stateMachine.currentTrunkCommand = goToSpitPose
        timer.reset()
    }

    override fun execute() {
        if (RobotContainer.trunkSystem.isAtPose && RobotContainer.stateMachine.intakeState != IntakeState.Spitting) {
            RobotContainer.cannonSystem.ampSpit()
        }

        if (RobotContainer.stateMachine.noteState == NoteState.Empty && !timer.isRunning) {
            timer.start()
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(.5)
    }

    override fun end(interrupted: Boolean) {
//        RobotContainer.stateMachine.currentTrunkCommand = StowTrunkCommand()
        RobotContainer.cannonSystem.killIntake()
    }
}