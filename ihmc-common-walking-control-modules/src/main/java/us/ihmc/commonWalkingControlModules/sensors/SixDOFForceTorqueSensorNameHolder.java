package us.ihmc.commonWalkingControlModules.sensors;

public class SixDOFForceTorqueSensorNameHolder {

	private String leftFootSixDOFForceTorqueSensorName;
	private String rightFootSixDOFForceTorqueSensorName;
	private String leftHandSixDOFForceTorqueSensorName;
	private String rightHandSixDOFForceTorqueSensorName;
	private String pelvisSixDOFForceTorqueSensorName;
	private String torsoSixDOFForceTorqueSensorName;
	
	private String leftFootFTJointName;
	private String rightFootFTJointName;
	private String leftHandFTJointName;
	private String rightHandFTJointName;
	private String pelvisFTJointName;
	private String torsoFTJointName;
	

	public String getLeftFootSixDOFForceTorqueSensorName() {
		return leftFootSixDOFForceTorqueSensorName;
	}

	public void setLeftFootSixDOFForceTorqueSensorName(String leftFootSixDOFForceTorqueSensorName) {
		this.leftFootSixDOFForceTorqueSensorName = leftFootSixDOFForceTorqueSensorName;
	}

	public String getRightFootSixDOFForceTorqueSensorName() {
		return rightFootSixDOFForceTorqueSensorName;
	}

	public void setRightFootSixDOFForceTorqueSensorName(String rightFootSixDOFForceTorqueSensorName) {
		this.rightFootSixDOFForceTorqueSensorName = rightFootSixDOFForceTorqueSensorName;
	}

	public String getLeftHandSixDOFForceTorqueSensorName() {
		return leftHandSixDOFForceTorqueSensorName;
	}

	public void setLeftHandSixDOFForceTorqueSensorName(String leftHandSixDOFForceTorqueSensorName) {
		this.leftHandSixDOFForceTorqueSensorName = leftHandSixDOFForceTorqueSensorName;
	}

	public String getRightHandSixDOFForceTorqueSensorName() {
		return rightHandSixDOFForceTorqueSensorName;
	}

	public void setRightHandSixDOFForceTorqueSensorName(String rightHandSixDOFForceTorqueSensorName) {
		this.rightHandSixDOFForceTorqueSensorName = rightHandSixDOFForceTorqueSensorName;
	}

	public String getPelvisSixDOFForceTorqueSensorName() {
		return pelvisSixDOFForceTorqueSensorName;
	}
	
	public void setPelvisSixDOFForceTorqueSensorName(String pelvisSixDOFForceTorqueSensorName) {
		this.pelvisSixDOFForceTorqueSensorName = pelvisSixDOFForceTorqueSensorName;
	}
	
	public String getTorsoSixDOFForceTorqueSensorName() {
		return torsoSixDOFForceTorqueSensorName;
	}
	
	public void setTorsoSixDOFForceTorqueSensorName(String torsoSixDOFForceTorqueSensorName) {
		this.torsoSixDOFForceTorqueSensorName = torsoSixDOFForceTorqueSensorName;
	}

	public String getLeftFootFTJointName() {
		return leftFootFTJointName;
	}

	public void setLeftFootFTJointName(String leftFootFTJointName) {
		this.leftFootFTJointName = leftFootFTJointName;
	}

	public String getRightFootFTJointName() {
		return rightFootFTJointName;
	}

	public void setRightFootFTJointName(String rightFootFTJointName) {
		this.rightFootFTJointName = rightFootFTJointName;
	}

	public String getLeftHandFTJointName() {
		return leftHandFTJointName;
	}

	public void setLeftHandFTJointName(String leftHandFTJointName) {
		this.leftHandFTJointName = leftHandFTJointName;
	}

	public String getRightHandFTJointName() {
		return rightHandFTJointName;
	}

	public void setRightHandFTJointName(String rightHandFTJointName) {
		this.rightHandFTJointName = rightHandFTJointName;
	}

	public String getPelvisFTJointName() {
		return pelvisFTJointName;
	}

	public void setPelvisFTJointName(String pelvisFTJointName) {
		this.pelvisFTJointName = pelvisFTJointName;
	}

	public String getTorsoFTJointName() {
		return torsoFTJointName;
	}

	public void setTorsoFTJointName(String torsoFTJointName) {
		this.torsoFTJointName = torsoFTJointName;
	}
	
	public String[] getFTJointNames()
	{
		String[] ftJointNames = {leftFootFTJointName, rightFootFTJointName, leftHandFTJointName, rightHandFTJointName, pelvisFTJointName, torsoFTJointName};
		return ftJointNames;
	}
	
	public String getFTSensorNameByJointname(String jointName)
	{
	   if (jointName.equals(getLeftFootFTJointName()))
         return getLeftFootSixDOFForceTorqueSensorName();
	   
	   if (jointName.equals(getRightFootFTJointName()))
         return getRightFootSixDOFForceTorqueSensorName();
	   
	   if (jointName.equals(getLeftHandFTJointName()))
         return getLeftHandSixDOFForceTorqueSensorName();
	   
	   if (jointName.equals(getRightHandFTJointName()))
         return getRightHandSixDOFForceTorqueSensorName();
	   
	   if (jointName.equals(getPelvisFTJointName()))
         return getPelvisSixDOFForceTorqueSensorName();
	   
	   if (jointName.equals(getTorsoFTJointName()))
         return getTorsoSixDOFForceTorqueSensorName();	   
	   
	   return null;
	}
}
