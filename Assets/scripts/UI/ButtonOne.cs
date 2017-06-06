using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class ButtonOne : MonoBehaviour {

public void LoadScene(int level)
    {
       SceneManager.LoadScene(level);
    }

    public void quit()
    {
            Application.Quit();

    }
}
