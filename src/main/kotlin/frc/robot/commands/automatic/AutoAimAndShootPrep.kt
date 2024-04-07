package frc.robot.commands.automatic

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.ShooterState
import frc.robot.TrunkPose
import frc.robot.commands.trunk.HoldPositionGoToAngleTrunk
import frc.robot.constants.TrunkConstants

class AutoAimAndShootPrep : Command() {
    val trunkCommand: HoldPositionGoToAngleTrunk = HoldPositionGoToAngleTrunk(TrunkPose.SPEAKER)

    override fun initialize() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting
        RobotContainer.stateMachine.currentTrunkCommand = trunkCommand;
    }

    override fun execute() {
        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity()

        //Handle the cannon aiming component
        val shooterAngle = clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)
//        SmartDashboard.putBoolean("shot is possible?", shooterAngle == shotSetup.shooterAngle)
//        RobotContainer.trunkSystem.setShootingAngle(shooterAngle)
        trunkCommand.desiredAngle = shooterAngle

        //Handle the twisting component

        //Can we shoot?
//        if (RobotContainer.stateMachine.trunkReady && MiscCalculations.appxEqual(
//                        twistPIDController.setpoint,
//                        shotSetup.robotAngle,
//                        1.0
//                ) && !autoShoot.isScheduled
//        ) {
//            autoShoot.schedule()
//        }

    }

    override fun isFinished(): Boolean {
        return RobotContainer.stateMachine.noteState == NoteState.Empty
    }

    override fun end(interrupted: Boolean) {
//        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.HIGH_STOW)
    }
}
