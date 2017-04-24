using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public enum CarDriveType
{
	FrontWheelDrive,
	RearWheelDrive,
	FourWheelDrive
}

public class ControllerCar : MonoBehaviour 
{
	public CarDriveType carDriveType = CarDriveType.FourWheelDrive;
	public WheelCollider[] wheelColliders = new WheelCollider[4];
	public GameObject[] wheelMeshes = new GameObject[4];
	public Vector3 centreOfMassOffSet;
	public float maximumSteerAngle;
	[Range(0, 1)]
	[Tooltip("0 is raw physics, 1 the car will grip in the direction it is facing")]
	public float steerHelper;
	[Range(0, 1)]
	[Tooltip("0 is no traction control, 1 is full interferance")]
	public float tractionControl;
	public float fullTorqueOverAllWheels;
	public float reverseTorque;
	public float maxHandBreakTorque;
	public float downForce = 100;
	public float topSpeed = 200;
	//public static int noOfGears = 5;
	//public float revRangeBoundary = 1;
	public float splipLimit;
	public float brakeTorque;


	private Quaternion[] wheelMeshLocalRotation;
	private Vector3 prevPos, pos;
	private float steerAngle;
	//private int gearNum;
	//private float gearFactor;
	private float oldRotation;
	private float currentTorque;
	private Rigidbody rigidBody;
	private const float reversingThreshold = 0.01f;


   // public bool Skidding { get; private set; }
    public float BrakeInput { get; private set; }
    public float CurrentSteerAngle{ get { return steerAngle; }}
    public float CurrentSpeed{ get { return rigidBody.velocity.magnitude*2.23693629f; }}
    public float MaxSpeed{get { return topSpeed; }}
 //   public float Revs { get; private set; }
    public float AccelInput { get; private set; }


    private void Start()
    {
    	wheelMeshLocalRotation = new Quaternion[4];
    	for(int i = 0; i < 4; i++)
    	{
    		wheelMeshLocalRotation[i] = wheelMeshes[i].transform.localRotation;
    	}

    	wheelColliders[0].attachedRigidbody.centerOfMass = centreOfMassOffSet;

    	maxHandBreakTorque = float.MaxValue;

    	rigidBody = GetComponent<Rigidbody>();
    	currentTorque = fullTorqueOverAllWheels - (tractionControl * fullTorqueOverAllWheels);
    }

    public void Move(float steering, float accel, float footbreak, float handbrake)
    {
    	//Iterate the 4 wheels
    	for(int i = 0; i < 4; i++)
    	{
    		Quaternion quat;
    		Vector3 position;
    		//Get position and rotation for each wheel collider
    		wheelColliders[i].GetWorldPose(out position, out quat);
    		//Update the meshe's position and rotation with value previously acquired from GetWorldPose
    		wheelMeshes[i].transform.position = position;
    		wheelMeshes[i].transform.rotation = quat;
    	}

    	//Clamp input values
    	steering = Mathf.Clamp (steering, -1, 1);
    	AccelInput = accel = Mathf.Clamp(accel, 0, 1);
    	BrakeInput = footbreak = -1*Mathf.Clamp(footbreak, -1, 0);
    	handbrake = Mathf.Clamp(handbrake, 0, 1);

    	//Set the steering on the front wheels
    	steerAngle = steering * maximumSteerAngle;
    	wheelColliders[0].steerAngle = steerAngle;
    	wheelColliders[1].steerAngle = steerAngle;

    	SteerHelper();
    	ApplyDrive(accel, footbreak);
    	CarSpeed();

    	//Set the handbrake
    	if(handbrake > 0)
    	{
    		float handBrakeTorque = handbrake * maxHandBreakTorque;
    		wheelColliders[2].brakeTorque = wheelColliders[3].brakeTorque = handBrakeTorque;
    	}

	    //Only used for display / sound purposes, it is not used for power / steering behaviours 
        //CalculateRevs();
        //GearChanging();

    	AddDownForce();

       //Plays sound, produces smoke particles, leaves skids on the ground
        //CheckForWheelSpin();
        // crude traction control that reduces the power to wheel if the car is wheel spinning too much
    	TractionControl();

    }

    private void SteerHelper()
    {
    	for(int i = 0; i < 4; i++)
    	{
    		WheelHit wheelHit;
    		wheelColliders[i].GetGroundHit(out wheelHit);
    		//wheels aren't on the ground
    		if(wheelHit.normal == Vector3.zero) return;
    	}

    	//This is neeeded to void Gimbal lock issues
    	if(Mathf.Abs(oldRotation  - transform.eulerAngles.y) < 10f)
    	{
    		float turnAdjust = (transform.eulerAngles.y - oldRotation) * steerHelper;
    		Quaternion velRotation = Quaternion.AngleAxis(turnAdjust, Vector3.up);
    		rigidBody.velocity = velRotation * rigidBody.velocity;
    	}
    	oldRotation = transform.eulerAngles.y;
    }

    private void ApplyDrive(float accel, float footbreak)
    {
    	float thrustTorque;

    	switch(carDriveType)
    	{
    		//In Four wheel drive the thrust is evenly split among all four wheels
    		case CarDriveType.FourWheelDrive:
	    		thrustTorque = accel * (currentTorque / 4);
	    		for(int i = 0; i < 4; i++)
	    		{
	    			wheelColliders[i].motorTorque = thrustTorque;
	    		}
	    		break;
	    	//Force is applied to the front wheels
	    	case CarDriveType.FrontWheelDrive:
	    		thrustTorque = accel * (currentTorque / 2);
	    		wheelColliders[0].motorTorque = wheelColliders[1].motorTorque = thrustTorque;
	    		break;
	    	//Force is applied to the back wheels
	    	case CarDriveType.RearWheelDrive:
	    		thrustTorque = accel * (currentTorque / 2);
	    		wheelColliders[2].motorTorque = wheelColliders[3].motorTorque = thrustTorque;
	    		break;
    	}

    	for(int i = 0; i < 4; i++)
    	{
    		if(CurrentSpeed > 5 && Vector3.Angle(transform.forward, rigidBody.velocity) < 50)
    		{
    			wheelColliders[i].brakeTorque = brakeTorque * footbreak;
    		}
    		else if(footbreak > 0)
    		{
    			wheelColliders[i].brakeTorque = 0;
    			wheelColliders[i].motorTorque = -reverseTorque * footbreak;
    		}
	
    	}   
     }

     private void CarSpeed()
     {
     	float speed = rigidBody.velocity.magnitude;

     	speed *= 3.6f;
     	if(speed > topSpeed)
     	{
     		rigidBody.velocity = (topSpeed / 3.6f) * rigidBody.velocity.normalized;
     	}
     }

     private void AddDownForce()
     {
     	wheelColliders[0].attachedRigidbody.AddForce(-transform.up * downForce * wheelColliders[0].attachedRigidbody.velocity.magnitude);
     }

     private void TractionControl()
     {
     	WheelHit wheelHit;

     	switch(carDriveType)
     	{
     		case CarDriveType.FourWheelDrive:
     			for(int i = 0; i < 4; i++)
     			{
     				wheelColliders[i].GetGroundHit(out wheelHit);
     				AdjustTorque(wheelHit.forwardSlip);
     			}
     			break;
     		case CarDriveType.RearWheelDrive:
     			wheelColliders[2].GetGroundHit(out wheelHit);
     			AdjustTorque(wheelHit.forwardSlip);

     			wheelColliders[3].GetGroundHit(out wheelHit);
     			AdjustTorque(wheelHit.forwardSlip);
     			break;
     		case CarDriveType.FrontWheelDrive:
     			wheelColliders[0].GetGroundHit(out wheelHit);
     			AdjustTorque(wheelHit.forwardSlip);

     			wheelColliders[1].GetGroundHit(out wheelHit);
     			AdjustTorque(wheelHit.forwardSlip);
     			break;
     	}
     }

     private void AdjustTorque(float forwardSlip)
     {
     	if(forwardSlip >= splipLimit && currentTorque >= 0)
     	{
     		currentTorque -= 10 * tractionControl;
     	}
     	else
     	{
     		currentTorque += 10 * tractionControl;
     		if(currentTorque > fullTorqueOverAllWheels) currentTorque = fullTorqueOverAllWheels;
     	}
     }
}
