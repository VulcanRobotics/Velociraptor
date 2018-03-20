package org.team1218.lib.trajectory;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.Trajectory.Segment;

public class TextfilePathSerDer{

	public static void serialize(Path path, File file) throws IOException{
		BufferedWriter writer = new BufferedWriter(new FileWriter(file));
		writer.write(path.getName() + "\n");
		path.goLeft();
		writer.write(path.getLeftWheelTrajectory().getNumSegments() + "\n");
		writeTrajectory(path.getLeftWheelTrajectory(),writer);
		writeTrajectory(path.getRightWheelTrajectory(),writer);
		
	}
	
	protected static void writeTrajectory(Trajectory traj, BufferedWriter writer) throws IOException {
		 for (int i = 0; i < traj.getNumSegments(); ++i) {
		      Segment segment = traj.getSegment(i);
		      writer.write(String.format(
              "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", 
              segment.pos, segment.vel, segment.acc, segment.jerk,
              segment.heading, segment.dt, segment.x, segment.y));
		 }
	}

	public Path deserialize(String serialized) {
		// TODO Auto-generated method stub
		return null;
	}

}
