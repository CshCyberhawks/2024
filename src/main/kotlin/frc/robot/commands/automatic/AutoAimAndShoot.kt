package frc.robot.commands.automatic


import MiscCalculations
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.GoToPoseTrunk
import frc.robot.commands.trunk.HoldPoseTrunk
import frc.robot.commands.trunk.HoldPositionGoToAngleTrunk
import frc.robot.constants.DriveConstants
import frc.robot.constants.TargetingConstants
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

    }

    override fun execute() {
        SmartDashboard.putNumber("shooter angle", shooterAngle)
        trunkCommand.desiredAngle = shooterAngle

        if (RobotContainer.actuallyDoShoot && !autoShoot.isScheduled) {
            autoShoot.schedule()
        }
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)
        RobotContainer.actuallyDoShoot = false
    }
}