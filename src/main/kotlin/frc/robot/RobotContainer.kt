package frc.robot

import com.pathplanner.lib.auto.NamedCommands
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
import frc.robot.commands.automatic.AutoAimAndShoot
import frc.robot.commands.automatic.FloorIntakeAndSeek
import frc.robot.commands.AutoAmp
import frc.robot.commands.AutoIntake
import frc.robot.commands.automatic.AutoAimDumbTwistAndShoot
import frc.robot.commands.cannon.AutoSpit
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.GoToPoseTrunk
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
    val xboxController: CommandXboxController = CommandXboxController(2)

    val telemetry = TelemetryToggles()

    val trunkSystem = TrunkSystem(TrunkIOReal())

    val stateMachine: RobotStateMachine = RobotStateMachine()

    val cannonSystem: CannonSystem = CannonSystem(CannonIOReal())

    val autonomousCommand: Command = Commands.run({})

    var teleopSwerveCommand: Command = TeleopSwerveDriveCommand()

    val targetingSystem: TargetingSystem = TargetingSystem()

    val visionSystem: VisionSystem = VisionSystem()

    var actuallyDoShoot: Boolean = false
    var actuallyDoAmp: Boolean = false

//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()

    val autoStateManagementEnableButton: Boolean
        get() = SmartDashboard.getBoolean("Enable Automatic State Management", false)

    val robotActionSendable: SendableChooser<RobotAction> = SendableChooser<RobotAction>()
    val shootPositionSendable: SendableChooser<ShootPosition> = SendableChooser<ShootPosition>()
    val trunkPoseSendable: SendableChooser<TrunkPose> = SendableChooser<TrunkPose>()

    val swerveSystem: SwerveSystem = SwerveSystem()

    val intakeLimelight = "limelight-back"

    init {
        configureBindings()
//        configureAutoCommands()

        RobotAction.entries.forEach {
            robotActionSendable.addOption(it.name, it)
        }

        ShootPosition.entries.forEach {
            shootPositionSendable.addOption(it.name, it)
        }

        TrunkPose.entries.forEach {
            trunkPoseSendable.addOption(it.name, it)
        }
    }

    private fun configureBindings() {
        rightJoystick.button(3).onTrue(Commands.runOnce({
            when (stateMachine.robotAction) {
                RobotAction.Speaker -> {
                    if (stateMachine.shootPosition == ShootPosition.AutoAim) {
                        AutoAimAndShoot()
                    } else {
                        AutoAimAndShoot()
                    }
                };
                RobotAction.Amp -> AutoAmp();
                RobotAction.SourceIntake -> TODO("Not yet implemented");
                RobotAction.FloorIntake -> AutoIntake()
                RobotAction.Trap -> TODO("Not yet implemented")
                //Does literally nothing
                RobotAction.Chill -> println("*Hits blunt* Yoooooooo sup bra (currently in chill mode)")
            }
        }))


        xboxController.leftBumper().onTrue(Commands.runOnce({
            actuallyDoShoot = true

        }))
        xboxController.start().onTrue(Commands.runOnce({
            actuallyDoAmp = true
        }))
        rightJoystick.button(4).toggleOnTrue(FloorIntakeAndSeek())

        xboxController.b().toggleOnTrue(AutoIntake())
        xboxController.a().onTrue(AutoAmp())
        xboxController.y().onTrue(AutoAimDumbTwistAndShoot())
        xboxController.x().onTrue(Commands.runOnce({ stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW) }))
        xboxController.rightBumper().toggleOnTrue(AutoSpit())

        leftJoystick.button(2).whileTrue(FloorIntakeAndSeek())


//    private fun configureAutoCommands() {
//        NamedCommands.registerCommand("FloorIntakeAndSeek", FloorIntakeAndSeek())
//    }

//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()
//        SmartDashboard.putData("Auto Chooser", autoChooser)
    }

}

