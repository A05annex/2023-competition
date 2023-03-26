package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.Mk4NeoModule;
import org.a05annex.util.Utl;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static frc.robot.Constants.ROBOT_SETTINGS;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class TestSpeedCacheSwerve {

    SpeedCachedSwerve getInitializedSCS() {
        SpeedCachedSwerve SCS = SpeedCachedSwerve.getInstance();
        A05Constants.RobotSettings cc = ROBOT_SETTINGS[0];
        SCS.setDriveGeometry(cc.m_length,cc.m_width,
                cc.m_rf, cc.m_rr, cc.m_lf, cc.m_lr,
                cc.m_maxSpeedCalibration);
        return SCS;
    }

    @Test
    @DisplayName("test cacheConfiguration")
    void TestConfiguration() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        A05Constants.RobotSettings cc = ROBOT_SETTINGS[0];
        assertEquals(cc.m_length, SCS.getDriveLength());
        assertEquals(cc.m_width, SCS.getDriveWidth());
        assertEquals(Mk4NeoModule.MAX_METERS_PER_SEC * cc.m_maxSpeedCalibration, SCS.getMaxMetersPerSec());
        double driveDiagonal = Utl.length(cc.m_length, cc.m_width);
        assertEquals(SCS.getMaxMetersPerSec() / (0.5 * driveDiagonal), SCS.getMaxRadiansPerSec());
    }

    @Test
    @DisplayName("test setCacheLength")
    void TestSetCacheLength() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        SCS.setCacheLength(10);
         assertEquals(10, SCS.getCacheLength());

    }

    @Test
    @DisplayName("test recordControlRequests")
    void TestRecordControlRequests() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        SCS.setCacheLength(10);
        for (int i = 0; i < 10; i++) {
            double currentTime = Timer.getFPGATimestamp();
            SCS.swerveDriveComponents((i * 0.1) - 0.5, (i * 0.1) - 0.9, (i * 0.1) - 0.1);
            assertEquals(i, SCS.mostRecentControlRequest);
            assertEquals((i * 0.1) - 0.5, SCS.controlRequests[SCS.mostRecentControlRequest].forward);
            assertEquals((i * 0.1) - 0.9, SCS.controlRequests[SCS.mostRecentControlRequest].strafe);
            assertEquals((i * 0.1) - 0.1, SCS.controlRequests[SCS.mostRecentControlRequest].rotation);
            assertTrue(currentTime <= SCS.controlRequests[SCS.mostRecentControlRequest].timeStamp);

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        assertEquals(9, SCS.mostRecentControlRequest);
    }

    @Test
    @DisplayName("test getPositionSinceTime")
    void TestGetPositionSinceTime() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        SCS.addControlRequest(0.2, 0.1, 0.3, 4.0);
        SCS.addControlRequest(0.2, 0.1, -0.1, 4.02);
        SCS.addControlRequest(0.2, 0.1, -0.1, 4.04);
        SCS.addControlRequest(0.2, 0.1, -0.1, 4.06);
        SCS.addControlRequest(0.2, 0.1, 0.3, 4.08);
        SpeedCachedSwerve.RobotRelativePosition position =
                SCS.getRobotRelativePositionSince(4.10, 4.03);
        //assertEquals(.06 * 0.2 * SCS.getMaxMetersPerSec(), position.forward);
    }
}
