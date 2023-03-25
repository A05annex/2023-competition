package frc.robot.subsystems;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class TestSpeedCacheSwerve {
    @Test
    @DisplayName("test setCacheLength")
    void TestSetCacheLength() {
        SpeedCachedSwerve SCS = SpeedCachedSwerve.getInstance();
        SCS.setCacheLength(10);
        assertEquals(10, SCS.getCacheLength());
    }

    @Test
    @DisplayName("test recordControlRequests")
    void TestRecordControlRequests() {
        SpeedCachedSwerve SCS = SpeedCachedSwerve.getInstance();
        SCS.setCacheLength(10);
        for (int i = 0; i < 10; i++) {
            long currentTime = System.currentTimeMillis();
            SCS.swerveDriveComponents(i * 0.1 - 0.5, i * 0.1 - 0.9, i * 0.1 - 0.1);
            assertEquals(i, SCS.mostRecentControlRequest);
            assertEquals(i * 0.1 - 0.5, SCS.controlRequests[SCS.mostRecentControlRequest].forward);
            assertEquals(i * 0.1 - 0.9, SCS.controlRequests[SCS.mostRecentControlRequest].strafe);
            assertEquals(i * 0.1 - 0.1, SCS.controlRequests[SCS.mostRecentControlRequest].rotation);
            assertTrue(currentTime <= SCS.controlRequests[SCS.mostRecentControlRequest].time);

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        assertEquals(9, SCS.mostRecentControlRequest);
    }
}
