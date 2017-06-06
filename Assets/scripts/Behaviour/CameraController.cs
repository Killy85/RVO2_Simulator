using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#pragma warning disable CS3009 // Le type de base n'est pas conforme CLS
public class CameraController : MonoBehaviour
#pragma warning restore CS3009 // Le type de base n'est pas conforme CLS
{

    // At each frame update position of the camera 
    //according to users inputs
    void Update()
    {

        if (Input.GetKey(KeyCode.RightArrow))
        {
            transform.Rotate(new Vector3(0, 10 * Time.deltaTime, 0));
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            transform.Rotate(new Vector3(0, -10 * Time.deltaTime, 0));
        }

    }

}
