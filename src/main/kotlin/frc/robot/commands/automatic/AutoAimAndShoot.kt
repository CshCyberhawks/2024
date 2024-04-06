package frc.robot.commands.automatic

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.ShooterState
import frc.robot.TrunkPose
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.HoldPositionGoToAngleTrunk
import frc.robot.commands.trunk.StowTrunkCommand
import frc.robot.constants.TrunkConstants

class AutoAimAndShoot : Command() {
    val autoShoot: AutoShootCommand = AutoShootCommand()

    var shooterAngle = 0.0

    val trunkCommand: HoldPositionGoToAngleTrunk = HoldPositionGoToAngleTrunk(TrunkPose.SPEAKER)

    override fun initialize() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting
        RobotContainer.stateMachine.currentTrunkCommand = trunkCommand;

        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity()

        shooterAngle = clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)
        trunkCommand.desiredAngle = 65.0
    }

    override fun execute() {
        SmartDashboard.putNumber("shooter angle", shooterAngle)

        if (RobotContainer.actuallyDoShoot && !autoShoot.isScheduled) {
            autoShoot.schedule()
        }
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.currentTrunkCommand = StowTrunkCommand()
        RobotContainer.actuallyDoShoot = false
    }
}
