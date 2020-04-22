// Author: Youshaa Murhij
// Email: yosha.morheg@gmail.com

using UnityEngine;
using System.Collections;
using Leap;
using Leap.Unity;
using Leap.Unity.Attributes;


public class UR5Controller : MonoBehaviour {

    public GameObject RobotBase , Camera_Rig;
    public double[] jointValues = new double[6];
    public double[] gripperValues = {-35f, -35f, -35f};
    private GameObject[] jointList = new GameObject[6];
    private GameObject[] gripperList = new GameObject[3];
    private GameObject Right_Controller;
    private double[] upperLimit = { 360f, 360f, 360f, 360f, 360f, 360f };
    private double[] lowerLimit = { -360f, -360f, -360f, -360f, -360f, -360f };
    public static double[] xyz_ref;
    public static Mat target_pose;
    public static RoboDK.Item ROBOT;
    public LeapServiceProvider provider;
    double x, y, z, X = 0, Y = 0, Z = 0 , Yaw =0, Pitch= 0, Roll = 0;
    double[] home_joints = {0.0f, -90.0f, -90.0f, 0.0f, 90.0f, 0.0f};
    double Factor_LM = 30, Factor_VR = 100; 
    bool Gripper_On = false;
    bool Leap_On = false;
    bool VR_controllers_On = true;
    int extendedFingers = 0;
    double[] VR_Init_Pos = {0.0f, 0.0f, 0.0f};

    
    // Use this for initialization
    void Start () {
        initializeJoints();
        RoboDK RDK = new RoboDK();
        ROBOT = RDK.ItemUserPick("Select a robot", RoboDK.ITEM_TYPE_ROBOT);
        RDK.setRunMode(RoboDK.RUNMODE_SIMULATE);
        ROBOT.MoveJ(home_joints);
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
        //Setting inial right controller 3D position
        VR_Init_Pos[0] = Right_Controller.transform.position.z;
        VR_Init_Pos[1] = Right_Controller.transform.position.x;
        VR_Init_Pos[2] = Right_Controller.transform.position.y; 
        //Debug.Log(VR_Init_Pos[0] + "  " + VR_Init_Pos[1] + "  " + VR_Init_Pos[2]);
	}
	
	// Update is called once per frame
	void LateUpdate () {
        if (Leap_On)
        {
            Frame frame = provider.CurrentFrame;
            if (frame != null)
            {
                if (frame.Hands.Count > 0)
                {
                    Hand hand = frame.Hands[0];
                    X = hand.PalmPosition.z;
                    Y = hand.PalmPosition.x;
                    Z = hand.PalmPosition.y;
                    Roll = hand.Rotation.x * 180 * 7 / 22;
                    Pitch = hand.Rotation.y * 180 * 7 / 22;
                    Yaw = hand.Rotation.z * 180 * 7 / 22;  
                    extendedFingers = 0;
                    for (int f = 0; f < hand.Fingers.Count; f++)  
                    {   //Check gripper State:
                        Finger digit = hand.Fingers [f];
                        if (digit.IsExtended) 
                        extendedFingers++;
                    }
                }
            }
            x = xyz_ref[0] + X * Factor_LM;
            y = xyz_ref[1] - Y * Factor_LM;
            z = xyz_ref[2] + Z * Factor_LM;
        }
        else if (VR_controllers_On)
        {
            X = Right_Controller.transform.position.z - VR_Init_Pos[0];
            Y = Right_Controller.transform.position.x - VR_Init_Pos[1];
            Z = Right_Controller.transform.position.y - VR_Init_Pos[2];
            Roll  = Right_Controller.transform.rotation.x * 180 * 7 / 22;
            Pitch = Right_Controller.transform.rotation.y * 180 * 7 / 22;
            Yaw   = Right_Controller.transform.rotation.z * 180 * 7 / 22;
            x = xyz_ref[0] + X * Factor_VR;
            y = xyz_ref[1] - Y * Factor_VR;
            z = xyz_ref[2] + Z * Factor_VR;
        }
        //Debug.Log(Yaw);       
        target_pose.setPos(x, y, z);
        ROBOT.MoveL(target_pose);
        jointValues = ROBOT.Joints();
        // Mapping between system coordinates in unity and Leapmotion
        jointValues[0]*= -1;
        jointValues[1]+= 90;
        jointValues[3]+= 90;
        jointValues[3]-= Roll; 
        jointValues[4]*= -1;
        //jointValues[4]+= Yaw;
        jointValues[5]+= Pitch;

        for ( int i = 0; i < 6; i ++) {
            Vector3 currentRotation = jointList[i].transform.localEulerAngles;
            currentRotation.z = (float)jointValues[i];
            jointList[i].transform.localEulerAngles = currentRotation;
        }
        if (extendedFingers <= 1)
        {
            Gripper_On = true; 
            for ( int i = 0; i < 3; i ++) {
                Vector3 currentRotation_gripper = gripperList[i].transform.localEulerAngles;
                currentRotation_gripper.x = (float)gripperValues[i];
                gripperList[i].transform.localEulerAngles = currentRotation_gripper;
            }
        }
        else if (extendedFingers >= 3)
        {
            Gripper_On = false; 
            for ( int i = 0; i < 3; i ++) {
                Vector3 currentRotation_gripper = gripperList[i].transform.localEulerAngles;
                currentRotation_gripper.x = 0.0f;
                gripperList[i].transform.localEulerAngles = currentRotation_gripper;
            }
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
            //================Gripper==================
            else if (RobotChildren[i].name == "victor_right_gripper_fingerA_base") {
                gripperList[0] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "victor_right_gripper_fingerB_base") {
                gripperList[1] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "victor_right_gripper_fingerC_base") {
                gripperList[2] = RobotChildren[i].gameObject;
            }
        }
        var Controller_Obj = Camera_Rig.GetComponentsInChildren<Transform>();
        for (int i = 0; i < Controller_Obj.Length; i++) {
            if (Controller_Obj[i].name == "RightHand") {
                Right_Controller = Controller_Obj[i].gameObject;
                break;
            }
        }
    }

}
