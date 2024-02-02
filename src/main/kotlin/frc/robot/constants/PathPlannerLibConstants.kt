package frc.robot.constants

import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import frc.robot.constants.yagsl_configs.YAGSLConfig

object PathPlannerLibConstants {
    // TODO: figure out if we need to change anything here
    var replanningConfig = ReplanningConfig()

    // TODO: tune; values are bad
    var translationPID = PIDConstants(5.0, 0.0, 0.0)
    var rotationPID = PIDConstants(0.0
    )
    var pathPlannerConfig = HolonomicPathFollowerConfig(
        translationPID,
        rotationPID,
        4.2,
        1.0,
        replanningConfig,
    )
}