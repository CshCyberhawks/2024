package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.TeleopSwerveDriveCommand
import frc.robot.commands.UnBreakTheIK
import frc.robot.commands.automatic.AutoAimFromPosition
import frc.robot.commands.automatic.AutoAimShooter
import frc.robot.commands.automatic.FloorIntakeAndSeek
import frc.robot.commands.cannon.AutoAmp
import frc.robot.commands.cannon.AutoIntake
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.commands.cannon.AutoSpit
import frc.robot.subsystems.VisionSystem
import frc.robot.subsystems.cannon.CannonIOReal
import frc.robot.subsystems.cannon.CannonSystem
import frc.robot.subsystems.swerve.SwerveSystem
import frc.robot.subsystems.trunk.TrunkIOReal
import frc.robot.subsystems.trunk.TrunkSystem
import frc.robot.util.TargetingSystem
import frc.robot.util.TelemetryToggles

object RobotContainer {
    val leftJoystick: CommandJoystick = CommandJoystick(0)
    val rightJoystick: CommandJoystick = CommandJoystick(1)
    private val xboxController: CommandXboxController = CommandXboxController(2)

    val telemetry = TelemetryToggles()

    val trunkSystem = TrunkSystem(TrunkIOReal())

    val stateMachine: RobotStateMachine = RobotStateMachine()

    val cannonSystem: CannonSystem = CannonSystem(CannonIOReal())

    val autonomousCommand: Command = Commands.run({})

    val autoAimShooter: Command = AutoAimShooter()

    val autoAimFromPosition: Command = AutoAimFromPosition(stateMachine.shootPosition.position)

    val autoAimFromPresetPosition: Command = AutoAimFromPosition(Pose2d(Translation2d(2.89, 5.54), Rotation2d()))

    val autoAmp: Command = AutoAmp()

    val teleopSwerveCommand: Command = TeleopSwerveDriveCommand()

    val targetingSystem: TargetingSystem = TargetingSystem()

    val visionSystem: VisionSystem = VisionSystem()

//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()

    val robotActionSendable: SendableChooser<RobotAction> = SendableChooser<RobotAction>()
    val shootPositionSendable: SendableChooser<ShootPosition> = SendableChooser<ShootPosition>()
    val trunkPositionSendable: SendableChooser<TrunkPosition> = SendableChooser<TrunkPosition>()

    val swerveSystem: SwerveSystem = SwerveSystem()

//    val teleopSwerveCommand: Command = if (DriveConstants.TWO_JOYSTICKS) {
//        swerveSystem.driveCommand({
//            MiscCalculations.calculateDeadzone(
//                rightJoystick.x, DriveConstants.TELEOP_DEADZONE_X
//            )
//        }, {
//            MiscCalculations.calculateDeadzone(
//                rightJoystick.y, DriveConstants.TELEOP_DEADZONE_Y
//            )
//        }, {
//            MiscCalculations.calculateDeadzone(leftJoystick.x, DriveConstants.TELEOP_DEADZONE_TWIST_TWO_JOY)
//        })
//    } else {
//        swerveSystem.driveCommand({
//            MiscCalculations.calculateDeadzone(
//                rightJoystick.x, DriveConstants.TELEOP_DEADZONE_X
//            )
//        }, {
//            MiscCalculations.calculateDeadzone(
//                rightJoystick.y, DriveConstants.TELEOP_DEADZONE_Y
//            )
//        }, {
//            MiscCalculations.calculateDeadzone(rightJoystick.twist, DriveConstants.TELEOP_DEADZONE_TWIST_ONE_JOY)
//        })
//    }
    val intakeLimelight = "limelight-back"

    val autoStateManagementEnableButton: Boolean
        get() = SmartDashboard.getBoolean("Enable Automatic State Management", false)

    init {
        configureBindings()

        RobotAction.entries.forEach {
            robotActionSendable.addOption(it.name, it)
        }

        ShootPosition.entries.forEach {
            shootPositionSendable.addOption(it.name, it)
        }

        TrunkPosition.entries.forEach {
            trunkPositionSendable.addOption(it.name, it)
        }

        teleopSwerveCommand.addRequirements(swerveSystem)
        autoAimFromPosition.addRequirements(trunkSystem)
        autoAimFromPresetPosition.addRequirements(trunkSystem)
        autoAimShooter.addRequirements(trunkSystem)
        autoAmp.addRequirements(cannonSystem)
    }

    private fun configureBindings() {
//        xboxController.a().toggleOnTrue(AutoIntake())
//        xboxController.x().toggleOnTrue(AutoShootCommand())
////        xboxController.x().onTrue(Commands.runOnce({
////            println("x button pressed")
////            cannonSystem.shoot()
////        }))
//        xboxController.b().onTrue(Commands.runOnce({
//            cannonSystem.killShooter()
//        }))
//        xboxController.y().toggleOnTrue(AutoAmp())
//        //MURDER...KILL IT ALL
//        xboxController.start().onTrue(Commands.runOnce({
//            cannonSystem.killShooter()
//            cannonSystem.killIntake()
//        }))

        rightJoystick.button(3).onTrue(Commands.runOnce({
            when (stateMachine.robotAction) {
                RobotAction.Speaker -> {
                    if (stateMachine.shootPosition == ShootPosition.AutoAim) {
                        autoAimShooter
                    } else {
                        autoAimFromPosition
                    }
                }

                RobotAction.Amp -> autoAmp
                RobotAction.SourceIntake -> TODO("Not yet implemented")
                RobotAction.FloorIntake -> AutoIntake()
                RobotAction.Trap -> TODO("Not yet implemented")
                //Does literally nothing
                RobotAction.Chill -> println("*Hits blunt* Yoooooooo sup bra (currently in chill mode)")
            }
        }))

        rightJoystick.button(4).toggleOnTrue(FloorIntakeAndSeek())
        rightJoystick.button(2).onTrue(Commands.runOnce({ swerveSystem.zeroGyro() }, swerveSystem))

        xboxController.x().onTrue(Commands.runOnce({
            stateMachine.targetTrunkPose = TrunkPosition.STOW
        }))
        xboxController.back().onTrue(Commands.runOnce({
            trunkSystem.STOP()
        }))
        xboxController.y().onTrue(UnBreakTheIK())
        xboxController.b().toggleOnTrue(AutoIntake())
        xboxController.leftBumper().onTrue(autoAimFromPresetPosition)
        xboxController.a().onTrue(AutoAimShooter())
        xboxController.rightBumper().toggleOnTrue(AutoSpit())
        xboxController.leftTrigger().onTrue(AutoShootCommand().onlyIf {
            autoAimShooter.isScheduled || autoAimFromPosition.isScheduled
        })
        xboxController.leftStick().whileTrue(
            Commands.runOnce({
                if (stateMachine.trunkState == TrunkState.MANUAL) {
                    trunkSystem.elevate(
                        -xboxController.leftY
                    )
                    trunkSystem.rotate(-xboxController.rightY * 0.1)
                }
            }, trunkSystem)
        )
        xboxController.start().onTrue(Commands.runOnce({ cannonSystem.ampSpit()}, cannonSystem))

        // axis 0 = x, axis 1 = y
//        val twoJoysticks = true
//        if (twoJoysticks) {
//            leftJoystick.axisGreaterThan(0, )
//        } else {
//
//        }
        leftJoystick.button(2).onTrue(FloorIntakeAndSeek())
    }

//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()
//        SmartDashboard.putData("Auto Chooser", autoChooser)

}

