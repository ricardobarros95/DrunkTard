using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;

[RequireComponent(typeof(ControllerCar))]
public class InputCar : MonoBehaviour {

    private ControllerCar car;

    private void Awake()
    {
        car = GetComponent<ControllerCar>();
    }

    private void FixedUpdate()
    {
        float h = CrossPlatformInputManager.GetAxis("Horizontal");
        float v = CrossPlatformInputManager.GetAxis("Vertical");
#if !MOBILE_INPUT
        float handbrake = CrossPlatformInputManager.GetAxis("Jump");
        car.Move(h, v, v, handbrake);
#else
            car.Move(h, v, v, 0f);
#endif
    }
}
