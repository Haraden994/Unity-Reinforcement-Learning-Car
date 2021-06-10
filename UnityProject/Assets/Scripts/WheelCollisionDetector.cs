using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelCollisionDetector : MonoBehaviour
{
    private CarAgent myCar;
    private WheelCollider _wc;
    private WheelHit _wh;
    
    private void Awake()
    {
        myCar = gameObject.GetComponentInParent<CarAgent>();
        _wc = GetComponent<WheelCollider>();
    }

    private void Update()
    {
        _wc.GetGroundHit(out _wh);
        if (_wh.collider.gameObject.CompareTag("wall"))
        {
            myCar.WheelCollided(); 
        }
    }
}
