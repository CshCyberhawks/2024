package frc.robot.commands.automatic

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.ShooterState
import frc.robot.TrunkPose
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.HoldPositionGoToAngleTrunk
import frc.robot.util.Timer

class AutoAmpShoot : Command() {
    val timer = Timer()

    var shooterAngle = 0.0

    val trunkCommand: HoldPositionGoToAngleTrunk = HoldPositionGoToAngleTrunk(TrunkPose.AMP_SHOOTING)

    override fun initialize() {
        RobotContainer.stateMachine.shooterState = ShooterState.Amping
        RobotContainer.stateMachine.currentTrunkCommand = trunkCommand;
        trunkCommand.desiredAngle = 50.0
        RobotContainer.actuallyDoShoot = false
    }

    override fun execute() {
//        if (RobotContainer.actuallyDoShoot && !autoShoot.isScheduled) {
//            autoShoot.schedule()
//        }
        if (RobotContainer.actuallyDoAmp) {
            RobotContainer.cannonSystem.feed()
            timer.start()
        }
    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Empty && timer.hasElapsed(.5)
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)
        RobotContainer.actuallyDoShoot = false
        RobotContainer.cannonSystem.killShooter()
        RobotContainer.cannonSystem.killIntake()
    }
}
