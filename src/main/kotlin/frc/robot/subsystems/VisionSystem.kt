package frc.robot.subsystems

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.LimelightHelpers
import frc.robot.RobotContainer
import frc.robot.constants.FieldConstants
import frc.robot.constants.LimelightConstants
import frc.robot.util.AllianceFlip
import frc.robot.util.visualiztion.Field2d

class VisionSystem {
//    val limelightNames: Array<String> =
//        arrayOf("limelight-left", "limelight-right", "limelight-rightsi", "limelight-leftsi", "limelight-center")

    val limelightNames: Array<String> =
        arrayOf("limelight-left", "limelight-right")


    val sideLimelightNames: Array<String> = arrayOf("limelight-rightsi", "limelight-leftsi")

    fun updateOdometryFromDisabled() {

        var namesToSearch: Array<String>;


        if (SmartDashboard.getBoolean("use new limelights", false)) {
            namesToSearch = limelightNames.plus(sideLimelightNames)
        } else {
            namesToSearch = limelightNames
        }


        for (llName in namesToSearch) {

            if (DriverStation.getAlliance().isEmpty) {
//                println("DS alliance is empty; skipping vision")
                return
            }


            var llMeasure: LimelightHelpers.PoseEstimate =
                if (DriverStation.getAlliance().isPresent && DriverStation.getAlliance()
                        .get() == DriverStation.Alliance.Blue
                ) {
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName)
                } else {
                    LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(llName)
                }
//            else {
//                println("DS alliance invalid; skipping vision")
//                return
//            }

            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                llMeasure = LimelightHelpers.PoseEstimate(
                    Pose2d(
                        Translation2d(
                            FieldConstants.fieldLength - llMeasure.pose.x,
                            FieldConstants.fieldWidth - llMeasure.pose.y
                        ),
                        llMeasure.pose.rotation.plus(Rotation2d(Math.PI))
                    ),
                    llMeasure.timestampSeconds,
                    llMeasure.latency,
                    llMeasure.tagCount,
                    llMeasure.tagSpan,
                    llMeasure.avgTagDist,
                    llMeasure.avgTagArea,
                    llMeasure.rawFiducials
                )
            }
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
//                    println("Updating with LL ${llName}: X = " + llMeasure.pose.x + " Y = " + llMeasure.pose.y)
                    RobotContainer.swerveSystem.driveTrain.addVisionMeasurement(
                        llMeasure.pose,
                        llMeasure.timestampSeconds
                    )
                }
            }
        }
    }

    fun updateOdometry(tagCount: Int, poseDifferenceCheck: Boolean) {

        var namesToSearch: Array<String>;


        if (SmartDashboard.getBoolean("use new limelights", false)) {
            namesToSearch = limelightNames.plus(sideLimelightNames)
        } else {
            namesToSearch = limelightNames
        }

        for (llName in namesToSearch) {
            if (DriverStation.getAlliance().isEmpty) {
//                println("DS alliance is empty; skipping vision")
                return
            }

            var llMeasure: LimelightHelpers.PoseEstimate =
                if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName)
                } else {
                    LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(llName)
                }
//            else {
//                println("DS alliance invalid; skipping vision")
//                return
//            }

//            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
//                llMeasure = LimelightHelpers.PoseEstimate(Pose2d(llMeasure.pose.translation, llMeasure.pose.rotation.plus(Rotation2d(Math.toRadians(180.0)))), llMeasure.timestampSeconds, llMeasure.latency, llMeasure.tagCount, llMeasure.tagSpan, llMeasure.avgTagDist, llMeasure.avgTagArea)
//            }
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                llMeasure = LimelightHelpers.PoseEstimate(
                    Pose2d(
                        Translation2d(
                            FieldConstants.fieldLength - llMeasure.pose.x,
                            FieldConstants.fieldWidth - llMeasure.pose.y
                        ),
                        llMeasure.pose.rotation.plus(Rotation2d(Math.PI))
                    ),
                    llMeasure.timestampSeconds,
                    llMeasure.latency,
                    llMeasure.tagCount,
                    llMeasure.tagSpan,
                    llMeasure.avgTagDist,
                    llMeasure.avgTagArea,
                    llMeasure.rawFiducials
                )
            }
            if (llMeasure.tagCount >= tagCount && llMeasure.pose.x != 0.0 && llMeasure.pose.y != 0.0) {
                val poseDifference =
                    llMeasure.pose.translation.getDistance(RobotContainer.swerveSystem.getSwervePose().translation)
                if (!poseDifferenceCheck || poseDifference < LimelightConstants.MAX_DISTANCE_DIFFERENCE_METERS) {
                    val distanceToTag = llMeasure.avgTagDist

                    if (distanceToTag < LimelightConstants.MAX_TAG_DISTANCE_METERS) {
                        var xyStds: Double
                        var degStds: Double

                        if (llName == "limelight-rightsi") {
                            if (llMeasure.tagCount >= 2) {
                                xyStds = 1.0
                                degStds = 12.0
                            } else if (llMeasure.avgTagArea > 0.8 && poseDifference < 0.5) {
                                xyStds = 2.0
                                degStds = 30.0
                            } else if (llMeasure.avgTagArea > 0.1 && poseDifference < 0.3) {
                                xyStds = 4.0
                                degStds = 50.0
                            } else {
                                xyStds = 6.0
                                degStds = 80.0
                            }
                        } else {
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
                        }

                        RobotContainer.swerveSystem.driveTrain.setVisionMeasurementStdDevs(
                            VecBuilder.fill(xyStds, xyStds, Math.toRadians(degStds))
                        )
//                        println("updating odometry with ll")
//                        println("Updating with LL ${llName}: X = " + llMeasure.pose.x + " Y = " + llMeasure.pose.y)

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
