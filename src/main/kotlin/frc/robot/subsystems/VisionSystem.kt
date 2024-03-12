package frc.robot.subsystems

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.LimelightHelpers
import frc.robot.RobotContainer
import frc.robot.constants.LimelightConstants

class VisionSystem {
    val limelightNames: Array<String> = arrayOf("limelight-left", "limelight-right")

    fun updateOdometryFromDisabled() {
        for (llName in limelightNames) {
            var llMeasure: LimelightHelpers.PoseEstimate

            if (DriverStation.getAlliance().isEmpty) {
                println("DS alliance is empty; skipping vision")
                return
            }

//            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            llMeasure = LimelightHelpers.getBotPoseEstimate_wpiRed(llName)
//            }
//            else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
//                llMeasure = LimelightHelpers.getBotPoseEstimate_wpiRed(llName)
//            }
//            else {
//                println("DS alliance invalid; skipping vision")
//                return
//            }

            if (llMeasure.pose.x != 0.0 && llMeasure.pose.y != 0.0) {
                val poseDifference =
                    llMeasure.pose.translation.getDistance(RobotContainer.swerveSystem.getSwervePose().translation)

                val distanceToTag = llMeasure.avgTagDist

                if (distanceToTag < LimelightConstants.MAX_TAG_DISTANCE_METERS) {
                    var xyStds: Double
                    var degStds: Double

                    if (llMeasure.tagCount >= 2) {
                        xyStds = 0.1
                        degStds = 6.0
                    } else if (llMeasure.avgTagArea > 0.8 && poseDifference < 0.5) {
                        xyStds = .3
                        degStds = 8.0
                    } else if (llMeasure.avgTagArea > 0.1 && poseDifference < 0.3) {
                        xyStds = .5
                        degStds = 13.0
                    } else {
                        xyStds = .8
                        degStds = 25.0
                    }

                    RobotContainer.swerveSystem.driveTrain.setVisionMeasurementStdDevs(
                        VecBuilder.fill(xyStds, xyStds, Math.toRadians(degStds))
                    )
//                        println("updating odometry with ll")
                    RobotContainer.swerveSystem.driveTrain.addVisionMeasurement(
                        llMeasure.pose,
                        llMeasure.timestampSeconds
                    )
                }
            }
        }
    }

    fun updateOdometry(tagCount: Int, poseDifferenceCheck: Boolean) {
        for (llName in limelightNames) {
            var llMeasure: LimelightHelpers.PoseEstimate

            if (DriverStation.getAlliance().isEmpty) {
                println("DS alliance is empty; skipping vision")
                return
            }

//            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            llMeasure = LimelightHelpers.getBotPoseEstimate_wpiRed(llName)
//            }
//            else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
//                llMeasure = LimelightHelpers.getBotPoseEstimate_wpiRed(llName)
//            }
//            else {
//                println("DS alliance invalid; skipping vision")
//                return
//            }

            if (llMeasure.tagCount >= tagCount && llMeasure.pose.x != 0.0 && llMeasure.pose.y != 0.0) {
                val poseDifference =
                    llMeasure.pose.translation.getDistance(RobotContainer.swerveSystem.getSwervePose().translation)
                if (poseDifferenceCheck == false || poseDifference < LimelightConstants.MAX_DISTANCE_DIFFERENCE_METERS) {
                    val distanceToTag = llMeasure.avgTagDist

                    if (distanceToTag < LimelightConstants.MAX_TAG_DISTANCE_METERS) {
                        var xyStds: Double
                        var degStds: Double

                        if (llMeasure.tagCount >= 2) {
                            xyStds = 0.5
                            degStds = 6.0
                        } else if (llMeasure.avgTagArea > 0.8 && poseDifference < 0.5) {
                            xyStds = 1.0
                            degStds = 12.0
                        } else if (llMeasure.avgTagArea > 0.1 && poseDifference < 0.3) {
                            xyStds = 2.0
                            degStds = 30.0
                        } else {
                            xyStds = 4.0
                            degStds = 50.0
                        }

                        RobotContainer.swerveSystem.driveTrain.setVisionMeasurementStdDevs(
                            VecBuilder.fill(xyStds, xyStds, Math.toRadians(degStds))
                        )
//                        println("updating odometry with ll")
                        RobotContainer.swerveSystem.driveTrain.addVisionMeasurement(
                            llMeasure.pose,
                            llMeasure.timestampSeconds
                        )
                    }
                }
            }
        }
    }
}