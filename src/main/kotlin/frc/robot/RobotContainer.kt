package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.ResetSwerveFieldForward
import frc.robot.subsystems.SwerveSystem

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
object RobotContainer {
    // The robot's subsystems and commands are defined here...
    val leftJoystick: CommandJoystick = CommandJoystick(0)
    val rightJoystick: CommandJoystick = CommandJoystick(1)
    val xboxController: CommandXboxController = CommandXboxController(2)

    val swerveSystem: SwerveSystem;

    lateinit var teleopSwerveDriveCommand: Command
    val autonomousCommand: Command = Commands.run({})

    val autoChooser: SendableChooser<Command>

    /**
     * The container for the robot.  Contains subsystems, IO devices, and commands.
     */
    init {
        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                swerveSystem = SwerveSystem()

            }
            Constants.Mode.SIM -> {
                // change these later
                swerveSystem = SwerveSystem()
            }
            Constants.Mode.REPLAY -> {
                // change these later
                swerveSystem = SwerveSystem()
            }
        }
        autoChooser = AutoBuilder.buildAutoChooser()
        // Configure the button bindings
        configureButtonBindings()

        SmartDashboard.putData("Auto Chooser", autoChooser)
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a
     * [edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings() {
        teleopSwerveDriveCommand = Commands.run(
            { swerveSystem.drive(Translation2d(leftJoystick.x, leftJoystick.y), leftJoystick.twist, true) },
            swerveSystem
        )
        rightJoystick.button(2).onTrue(ResetSwerveFieldForward())
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
}
