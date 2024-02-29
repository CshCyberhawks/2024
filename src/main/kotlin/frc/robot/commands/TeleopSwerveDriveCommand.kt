package frc.robot.commands

import MiscCalculations.calculateDeadzone
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants

class TeleopSwerveDriveCommand : Command() {
    init {
        addRequirements(RobotContainer.swerveSystem)
    }

    override fun execute() {
//        swerveDrive.drive
        val deadzoneX = .15
        val deadzoneY = .15
        val deadzoneTwist = .15
        val twoJoysticks = true

        val twist = if (twoJoysticks) {
            -RobotContainer.leftJoystick.x
        } else {
            RobotContainer.rightJoystick.twist
        }

//        println(twist)

        //Milan: trust me bro this'll work totally definitely please don't question it
        val throttle = ((RobotContainer.rightJoystick.throttle * -1) + 1) / 2

//        println(throttle)

        if (RobotContainer.rightJoystick.button(2).asBoolean) {
//            println("pressed reset")
            RobotContainer.swerveSystem.swerveDrive.zeroGyro()
        }
//        else {
//            println("not pressed reset")
//        }

        RobotContainer.swerveSystem.drive(
            Translation2d(
                /**(if (abs(RobotContainer.rightJoystick.x) > 0.15) {
                val inSpeed =
                if (RobotContainer.rightJoystick.x < 0.0) RobotContainer.rightJoystick.x + .15 else RobotContainer.rightJoystick.x - .15
                (inSpeed) * DriveConstants.MAX_SPEED
                } else 0.0),
                (if (abs(RobotContainer.rightJoystick.y) > 0.15) {
                val inSpeed =
                if (RobotContainer.rightJoystick.y < 0.0) RobotContainer.rightJoystick.y + .15 else RobotContainer.rightJoystick.y - .15
                (-inSpeed) * DriveConstants.MAX_SPEED
                } else 0.0)**/
                -calculateDeadzone(RobotContainer.rightJoystick.y, deadzoneX) * DriveConstants.MAX_SPEED * throttle,
                -calculateDeadzone(RobotContainer.rightJoystick.x, deadzoneY) * DriveConstants.MAX_SPEED * throttle
            ),
            calculateDeadzone(twist, deadzoneTwist) * throttle * DriveConstants.MAX_ANGLE_SPEED,
            true
        )
    }
}