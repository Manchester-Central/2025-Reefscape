package frc.robot.utils;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
public class FieldPointTests {
    double DELTA = 0.05;
    @Test
    public void testProcessorPose() {
        final FieldPoint proc = FieldPoint.processor;
        var blu_pose = proc.getBluePose(); // April Tag 3
        assertEquals(5.988, blu_pose.getTranslation().getX(), DELTA);
        assertEquals(0.003, blu_pose.getTranslation().getY(), DELTA);
        var red_pose = proc.getRedPose(); // April Tag 16
        assertEquals(11.561, red_pose.getTranslation().getX(), DELTA);
        assertEquals(8.056, red_pose.getTranslation().getY(), DELTA);
        final FieldPoint source_left = FieldPoint.leftSource;
        blu_pose = source_left.getBluePose(); // April Tag 2
        assertEquals(16.685, blu_pose.getTranslation().getX(), DELTA);
        assertEquals(7.398, blu_pose.getTranslation().getY(), DELTA);
        red_pose = source_left.getRedPose(); // April Tag 12
        assertEquals(0.852, red_pose.getTranslation().getX(), DELTA);
        assertEquals(0.656, red_pose.getTranslation().getY(), DELTA);
        final FieldPoint source_right = FieldPoint.rightSource;
        blu_pose = source_right.getBluePose(); // April Tag 1
        assertEquals(16.697, blu_pose.getTranslation().getX(), DELTA);
        assertEquals(0.6553, blu_pose.getTranslation().getY(), DELTA);
        red_pose = source_right.getRedPose(); // April Tag 13
        assertEquals(0.8512, red_pose.getTranslation().getX(), DELTA);
        assertEquals(7.396, red_pose.getTranslation().getY(), DELTA);
    }
}