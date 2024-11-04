package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.*
import frc.robot.commands.automatic.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.commands.cannon.AutoSpit
import frc.robot.commands.cannon.ShooterFeedCannon
import frc.robot.commands.simulation.SimTeleopSwerveDriveCommand
import frc.robot.commands.trunk.CalibrateTrunk
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.commands.trunk.GoToPoseTrunk
import frc.robot.commands.trunk.HoldPoseTrunk
import frc.robot.constants.DriveConstants
import frc.robot.constants.TargetingConstants
import frc.robot.constants.TrunkConstants
import frc.robot.subsystems.VisionSystem
import frc.robot.subsystems.cannon.CannonIOReal
import frc.robot.subsystems.cannon.CannonSystem
import frc.robot.subsystems.swerve.GenericSwerveSystem
import frc.robot.subsystems.swerve.SwerveSystemReal
import frc.robot.subsystems.swerve.SwerveSystemSim
import frc.robot.subsystems.swerve.SwerveTelemetry
import frc.robot.subsystems.trunk.TrunkIOReal
import frc.robot.subsystems.trunk.TrunkIOSim
import frc.robot.subsystems.trunk.TrunkSystem
import frc.robot.util.TargetingSystem
import frc.robot.util.TelemetryToggles
import frc.robot.util.betterToggleOnTrue

object RobotContainer {
    val robotType = RobotType.Real

    val leftJoystick: CommandJoystick = CommandJoystick(0)
    val rightJoystick: CommandJoystick = CommandJoystick(1)
    val xboxController: CommandXboxController = CommandXboxController(2)

    val telemetry = TelemetryToggles()

    val trunkSystem = TrunkSystem(
        when (robotType) {
            RobotType.Real -> TrunkIOReal()
            RobotType.Simulated -> TrunkIOReal()
        }
    )

    val stateMachine: RobotStateMachine = RobotStateMachine()

    val cannonSystem: CannonSystem = CannonSystem(CannonIOReal())

//    val climbLatch: Servo = Servo(TrunkConstants.CLIMB_LATCH_ID)

    val autonomousCommand: Command = Commands.run({})

    var teleopSwerveCommand: Command = when (robotType) {
        RobotType.Real -> TeleopSwerveDriveCommand()
        RobotType.Simulated -> SimTeleopSwerveDriveCommand()
    }

//    val climbCommand: Command = GoToPoseTrunk(TrunkPose.CLIMB).andThen(GoToPoseTrunk(TrunkPose.CLIMB_DOWN)).andThen({
//        //climbLatch.angle = 90.0 // tune before testing
//    }).alongWith(HoldPoseTrunk(TrunkPose.CLIMB_DOWN))

    val targetingSystem: TargetingSystem = TargetingSystem()

    val visionSystem: VisionSystem = VisionSystem()

    var actuallyDoShoot: Boolean = false
    var actuallyDoAmp: Boolean = false
    var actuallyDoClimb: Boolean = false

    val logger: SwerveTelemetry = SwerveTelemetry(DriveConstants.MAX_SPEED);


//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()

    val autoStateManagementEnableButton: Boolean
        get() = SmartDashboard.getBoolean("Enable Automatic State Management", false)

    val robotActionSendable: SendableChooser<RobotAction> = SendableChooser<RobotAction>()
    val shootPositionSendable: SendableChooser<ShootPosition> = SendableChooser<ShootPosition>()
    val trunkPoseSendable: SendableChooser<TrunkPose> = SendableChooser<TrunkPose>()

    val swerveSystem: GenericSwerveSystem = when (robotType) {
        RobotType.Real -> SwerveSystemReal()
        RobotType.Simulated -> SwerveSystemSim()
    }

    val intakeLimelight = "limelight-back"

    init {
        when (robotType) {
            RobotType.Real -> configureBindingsRobot()
            RobotType.Simulated -> configureBindingsSimulation()
        }
        configureAutoCommands()

        RobotAction.entries.forEach {
            robotActionSendable.addOption(it.name, it)
        }

        ShootPosition.entries.forEach {
            shootPositionSendable.addOption(it.name, it)
        }

        TrunkPose.entries.forEach {
            trunkPoseSendable.addOption(it.name, it)
        }

        robotActionSendable.setDefaultOption("FloorIntake", RobotAction.FloorIntake)
        SmartDashboard.putData("Robot Action", robotActionSendable)
    }

    private fun configureBindingsRobot() {
        rightJoystick.button(3).onTrue(Commands.runOnce({
            when (stateMachine.robotAction) {
                RobotAction.Speaker -> TeleopAimTwistAndShoot().schedule()
                RobotAction.Amp -> AutoAmp().schedule();
                RobotAction.SourceIntake -> println("Tried to source intake (not yet implemented)")
                RobotAction.FloorIntake -> AutoIntake().schedule()
                RobotAction.Trap -> println("Tried to trap score (not yet implemented)")
                RobotAction.Climb -> AutoClimbCommand().schedule()
                //Does literally nothing
                RobotAction.Chill -> println("*Hits blunt* Yoooooooo sup bra (currently in chill mode)")
            }
        }))

        leftJoystick.button(3).onTrue(Commands.runOnce({
            when (stateMachine.robotAction) {
                RobotAction.Speaker -> actuallyDoShoot = true
                RobotAction.Amp -> actuallyDoAmp = true
                RobotAction.SourceIntake -> println("Tried to trigger source intake (not yet implemented)")
                RobotAction.FloorIntake -> print("Tried to trigger floor intake (why?)")
                RobotAction.Trap -> println("Tried to trigger trap score (not yet implemented)")
                RobotAction.Climb -> actuallyDoClimb = true
                //Does literally nothing
                RobotAction.Chill -> println("*Hits blunt* Yoooooooo sup bra (currently in chill mode)")
            }
        }))

        leftJoystick.button(4).onTrue(Commands.runOnce({
            stateMachine.limelightReset = true
        }))
        leftJoystick.button(4).onFalse(Commands.runOnce({
            stateMachine.limelightReset = false
        }))

        xboxController.leftBumper().onTrue(Commands.runOnce({
            actuallyDoShoot = true
        }))
        xboxController.start().onTrue(Commands.runOnce({
            actuallyDoAmp = true
        }))
        rightJoystick.button(4).whileTrue(FloorIntakeAndSeek())

        // Align to amp
        rightJoystick.button(5).betterToggleOnTrue(AutoDriveToPose(Pose2d(1.82, 7.6, Rotation2d.fromDegrees(-90.0))))

//        xboxController.b().betterToggleOnTrue(AutoIntake())
        xboxController.b().betterToggleOnTrue(ShooterFeedCannon())
//        xboxController.a().onTrue(Commands.runOnce({
//            stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.CalibrationAngle)
//        }))
        xboxController.a().betterToggleOnTrue(AutoAmp())

//        xboxController.y().betterToggleOnTrue(TeleopAimDumbTwistAndShoot())
        xboxController.y().betterToggleOnTrue(AutoShootCommand())
//        xboxController.y().betterToggleOnTrue(AutoAimDumbTwistAndShoot())
        xboxController.povUp().onTrue(Commands.runOnce({ TargetingConstants.endpointZ += .01 }))
        xboxController.povDown().onTrue(Commands.runOnce({ TargetingConstants.endpointZ -= .01 }))
        xboxController.x().betterToggleOnTrue(TeleopMortarShoot())
        xboxController.rightBumper().onTrue(AutoSpit())
        xboxController.leftTrigger().betterToggleOnTrue(AutoClimbCommand())
        xboxController.rightTrigger().onTrue(Commands.runOnce({
            actuallyDoClimb = true
        }))
        xboxController.back()
            .onTrue(Commands.runOnce({ stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW) }))
//        leftJoystick.button(10).toggleOnTrue(KillTrunk())

//        leftJoystick.button(2).onTrue(Commands.runOnce({
//            stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.CalibrationAngle)
//        }))
//        rightJoystick.button(2).onTrue(Commands.runOnce({
//            stateMachine.intakeState = IntakeState.Spitting
//        }))
    }

    fun configureBindingsSimulation() {

    }

    private fun configureAutoCommands() {
        NamedCommands.registerCommand("AutoFloorIntakeAndSeek", AutoFloorIntakeAndSeek())
        NamedCommands.registerCommand("AutoIntake", AutoIntake())
        NamedCommands.registerCommand("AutoAimDumbTwistAndShoot", AutoAimDumbTwistAndShoot())
        NamedCommands.registerCommand(
            "Stow",
            Commands.runOnce({ stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW) })
        )
        NamedCommands.registerCommand("AutoAimAndShootPrep", AutoAimAndShootPrep())
        NamedCommands.registerCommand("CalibrateTrunk", CalibrateTrunk())
        NamedCommands.registerCommand("AutoSpinUpShooter", AutoSpinUpShooter())
    }

//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()
//        SmartDashboard.putData("Auto Chooser", autoChooser)


}

