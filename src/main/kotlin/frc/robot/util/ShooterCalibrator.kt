package frc.robot.util

/*
class ShooterCalibrator(val directory: String, val gun: GUNSystem) {
    var lastShot: Shot? = null
    fun readCsv(): List<Shot> {
        val inputStream = FileInputStream(directory)
        val reader = inputStream.bufferedReader()
        val header = reader.readLine()
        return reader.lineSequence()
            .filter { it.isNotBlank() }
            .map { line ->
                val list = line.split(",", ignoreCase = false, limit = 8).map { it.trim().toDouble() }
                Shot(
                    list[0],
                    list[1],
                    list[2],
                    list[3],
                    list[4],
                    list[5],
                    list[6],
                    list[7],
                )
            }.toList()
    }

    fun OutputStream.writeCsv(shots: List<Shot>) {
        val writer = bufferedWriter()
        if (!File(directory).exists()) {
            writer.write(""""x", "y", "vx", "vy", "robotAngle", "shooterAngle", "elevatorHeight", "leftPower", "rightPower"""")
            writer.newLine()
        }
        shots.forEach {
            writer.write(it.toCSV())
            writer.newLine()
        }
        writer.flush()
    }

    fun writeOut(shots: List<Shot>) {
        FileOutputStream(directory).apply {
            writeCsv(shots)
        }
    }

    fun shoot(shooterAngle: Double, leftPower: Double, rightPower: Double) {
        gun.shoot(shooterAngle, leftPower, rightPower)
        val botPose = RobotContainer.swerveSystem.swerveDrive.pose
        val v = RobotContainer.swerveSystem.swerveDrive.fieldVelocity
        lastShot = Shot(
            botPose.x,
            botPose.y,
            v.vxMetersPerSecond,
            v.vyMetersPerSecond,
            botPose.rotation.radians,
            shooterAngle,
            leftPower,
            rightPower,
        )
    }

    fun addLastShot() {

    }
}
 */