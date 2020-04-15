// Author: Long Qian
// Email: lqian8@jhu.edu

using UnityEngine;
using System.Collections;
using Leap;
using Leap.Unity;
using Leap.Unity.Attributes;

public class UR5Controller : MonoBehaviour {

    public GameObject RobotBase;
    public double[] jointValues = new double[6];
    private GameObject[] jointList = new GameObject[6];
    private double[] upperLimit = { 180f, 180f, 180f, 180f, 180f, 180f };
    private double[] lowerLimit = { -180f, -180f, -180f, -180f, -180f, -180f };
    public static double[] xyz_ref;
    public static Mat target_pose;
    public static RoboDK.Item ROBOT;
    public LeapServiceProvider provider;
    double x, y, z, X = 0, Y = 0, Z = 0;
    //double[] joints;
    double Factor_LM = 40; //400
    
    // Use this for initialization
    void Start () {
        initializeJoints();
        RoboDK RDK = new RoboDK();
        ROBOT = RDK.ItemUserPick("Select a robot", RoboDK.ITEM_TYPE_ROBOT);
        RDK.setRunMode(RoboDK.RUNMODE_SIMULATE);
        //Variables.ROBOT.MoveJ(home_joints);
        Mat frame = ROBOT.PoseFrame();
        //Mat tool = Variables.ROBOT.PoseTool();
        Mat pose_ref = ROBOT.Pose();
        target_pose = ROBOT.Pose();
        xyz_ref = target_pose.Pos();
        ROBOT.MoveJ(pose_ref);
        ROBOT.setPoseFrame(frame);  
        //Variables.ROBOT.setPoseTool(tool);   
        ROBOT.setSpeed(500);        
        ROBOT.setZoneData(5);
	}
	
	// Update is called once per frame
	void LateUpdate () {

        Frame frame = provider.CurrentFrame;
        if (frame != null)
        {
           if (frame.Hands.Count > 0)
           {
               Hand hand = frame.Hands[0];
               X = hand.PalmPosition.z;
               Y = hand.PalmPosition.x;
               Z = hand.PalmPosition.y;
           }
        x = xyz_ref[0] + X * Factor_LM;
        y = xyz_ref[1] - Y * Factor_LM;
        z = xyz_ref[2] + Z * Factor_LM;

        }
        target_pose.setPos(x, y, z);
        //ROBOT.MoveL(target_pose);
        jointValues = ROBOT.Joints();
        jointValues[0]*=-1;
        jointValues[1]+=90;
        jointValues[3]+=90;
        //jointValues[4]-=90;
        jointValues[4]*=-1;
        //jointValues[5]*=-1;

        for ( int i = 0; i < 6; i ++) {
            Vector3 currentRotation = jointList[i].transform.localEulerAngles;
            //Debug.Log(currentRotation);
            currentRotation.z = (float)jointValues[i];
            jointList[i].transform.localEulerAngles = currentRotation;
        }
	}

    void OnGUI() {
        int boundary = 20;

#if UNITY_EDITOR
        int labelHeight = 20;
        GUI.skin.label.fontSize = GUI.skin.box.fontSize = GUI.skin.button.fontSize = 20;
#else
        int labelHeight = 40;
        GUI.skin.label.fontSize = GUI.skin.box.fontSize = GUI.skin.button.fontSize = 40;
#endif
        GUI.skin.label.alignment = TextAnchor.MiddleLeft;
        for (int i = 0; i < 6; i++) {
            GUI.Label(new Rect(boundary + 400, boundary + ( i * 2 + 1 ) * labelHeight, labelHeight * 4 + 60, labelHeight), "Joint " + (i+1) + ": " + (int)jointValues[i] );
            jointValues[i] = GUI.HorizontalSlider(new Rect(boundary+ 400 + labelHeight * 4 + 60, boundary + (i * 2 + 1) * labelHeight + labelHeight / 4, labelHeight * 5, labelHeight), (float)jointValues[i], (float)lowerLimit[i], (float)upperLimit[i]);
        }
    }

    // Create the list of GameObjects that represent each joint of the robot
    void initializeJoints() {
        var RobotChildren = RobotBase.GetComponentsInChildren<Transform>();
        for (int i = 0; i < RobotChildren.Length; i++) {
            if (RobotChildren[i].name == "control0") {
                jointList[0] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "control1") {
                jointList[1] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "control2") {
                jointList[2] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "control3") {
                jointList[3] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "control4") {
                jointList[4] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "control5") {
                jointList[5] = RobotChildren[i].gameObject;
            }
        }
    }
}
