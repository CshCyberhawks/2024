package frc.robot.commands.automatic

import MiscCalculations
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.GoToPoseTrunk
import frc.robot.commands.trunk.HoldPositionGoToAngleTrunk
import frc.robot.constants.DriveConstants
import frc.robot.constants.TrunkConstants

class AutoAimDumbTwistAndShoot : Command() {
    val autoShoot: AutoShootCommand = AutoShootCommand()

    val trunkCommand: HoldPositionGoToAngleTrunk = HoldPositionGoToAngleTrunk(TrunkPose.SPEAKER)

    val twistPIDController: PIDController = PIDController(10.0, 0.0, 0.1)

    override fun initialize() {
        RobotContainer.stateMachine.driveState = DriveState.TranslationTeleop
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting
        RobotContainer.stateMachine.currentTrunkCommand = trunkCommand;

        twistPIDController.enableContinuousInput(0.0, 360.0);
    }

    override fun execute() {
        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocity()

        //Handle the cannon aiming component
        val shooterAngle = clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)
//        SmartDashboard.putBoolean("shot is possible?", shooterAngle == shotSetup.shooterAngle)
//        RobotContainer.trunkSystem.setShootingAngle(shooterAngle)
        trunkCommand.desiredAngle = shooterAngle


        //Handle the twisting component
        val driveTwist = twistPIDController.calculate(
                RobotContainer.swerveSystem.getSwervePose().rotation.degrees,
                shotSetup.robotAngle
        )
        SmartDashboard.putNumber("Shot angle", shotSetup.robotAngle)
        SmartDashboard.putNumber("Shot Drive twist", driveTwist)


        val driveTranslation = RobotContainer.swerveSystem.calculateJoyTranslation(
                RobotContainer.rightJoystick.x, RobotContainer.rightJoystick.y,
                RobotContainer.swerveSystem.calculateJoyThrottle(RobotContainer.leftJoystick.throttle),
                DriveConstants.TELEOP_DEADZONE_X,
                DriveConstants.TELEOP_DEADZONE_Y
        )

        RobotContainer.swerveSystem.driveTrain.applyRequest {
            RobotContainer.swerveSystem.drive.withVelocityX(driveTranslation.x).withVelocityY(driveTranslation.y)
                    .withRotationalRate(Math.toRadians(driveTwist))
        }.execute()


        //Can we shoot?
//        if (RobotContainer.stateMachine.trunkReady && MiscCalculations.appxEqual(
//                        twistPIDController.setpoint,
//                        shotSetup.robotAngle,
//                        1.0
//                ) && !autoShoot.isScheduled
//        ) {
//            autoShoot.schedule()
//        }
        if (RobotContainer.actuallyDoShoot && !autoShoot.isScheduled) {
            autoShoot.schedule()
        }
    }

    override fun isFinished(): Boolean {
//        return (autoShoot.isFinished) || RobotContainer.stateMachine.noteState == NoteState.Empty
//        return RobotContainer.stateMachine.noteState == NoteState.Empty
        return false
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)
        RobotContainer.actuallyDoShoot = false
        RobotContainer.stateMachine.driveState = DriveState.Teleop
    }
}