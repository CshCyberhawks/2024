package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface SwerveSystemIO {
    object SwerveSystemIOInputs : LoggableInputs {
        var givenTranslation: Translation2d = Translation2d()
        var givenRotation: Double = 0.0

        override fun toLog(table: LogTable) {
            table.put("givenTranslation", givenTranslation)
            table.put("givenRotation", givenRotation)
        }

        override fun fromLog(table: LogTable) {
//            robotAccel = table.get("robotAccel", robotAccel)[0]
//            robotVelocity = table.get("robotVelocity", robotVelocity)[0]
//            gyroRotation = table.get("gyroRotation", gyroRotation)[0]
//            robotPose = table.get("robotPose", robotPose)[0]
//
//            drivePositions = table.get("drivePositions", drivePositions)
//            driveVelocities = table.get("driveVelocities", driveVelocities)
//
//            turnAbsolutePositions = table.get("turnAbsolutePositions", turnAbsolutePositions)
//            turnPositions = table.get("turnPositions", turnPositions)
//            turnVelocities = table.get("turnVelocities", turnVelocities)
            givenTranslation = table.get("inputVelocity", givenTranslation)[0]
            givenRotation = table.get("inputRotation", givenRotation)
        }
    }

    fun updateInputs(inputs: SwerveSystemIOInputs) {}
}