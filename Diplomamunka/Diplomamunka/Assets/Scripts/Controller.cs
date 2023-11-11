using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography;
using UnityEngine;

public class Controller : MonoBehaviour
{
    [SerializeField] Transform egoCar;
    [SerializeField] Material pointCloudSkybox;
    [SerializeField] Material objMaterial;
    [SerializeField] Color[] pointColors;

    [SerializeField] bool dontScan;
    [SerializeField] bool isGeneratorScene;
    [SerializeField] int moveAndScanCount;
    [SerializeField] int displayFrame;

    Material defaultSkybox;
    LidarController lidarCont;
    MeshRenderer[] renderers;
    SkinnedMeshRenderer[] humanRenderers;
    TrainDataGenerator traindataGen;
    List<GameObject> generatedModels;
    bool renderersTurnedOff;
    bool processedShowed;
    bool displayedById;
    bool linesShowed;
    bool objectsShowed;

    void temp()
    {
        for (int i = 0; i < 100; i++)
        {
            int counter = 0;
            GameObject obj = Resources.Load<GameObject>("Generated_Models_" + i + "/processed_obj_0");
            while (obj != null)
            {
                var instance = (Instantiate(obj));
                instance.AddComponent<MeshFilter>();
                instance.GetComponent<MeshFilter>().mesh = obj.transform.GetChild(0)
                        .GetComponent<MeshFilter>().sharedMesh;
                instance.AddComponent<MeshRenderer>();
                Material m = Instantiate(objMaterial);
                m.color = pointColors[(counter + 1) % pointColors.Length];
                instance.GetComponent<MeshRenderer>().material = m;
                generatedModels.Add(instance);
                //instance.SetActive(false);
                counter++;
                obj = Resources.Load<GameObject>("Generated_Models_" + i + "/processed_obj_" + counter);
            }
            print(i + " " + counter);
        }
    }

    void Start()
    {
        traindataGen = FindObjectOfType<TrainDataGenerator>();
        generatedModels = new List<GameObject>();
        defaultSkybox = RenderSettings.skybox;
        lidarCont = FindObjectOfType<LidarController>();
        humanRenderers = FindObjectsOfType<SkinnedMeshRenderer>();
        temp();
        return;
        if (isGeneratorScene)
        {
            traindataGen.GenerateSceneTrigger();
        }
        else if(!dontScan) 
        {
            if(moveAndScanCount > 0)
            {
                StartCoroutine(MultipleScan());
            }
            else
                lidarCont.Scan(false, -1);
        }
        if (dontScan)
        {
            lidarCont.ReadRawData(displayFrame);
        }
        lidarCont.ReadProcessedData();
        ImportObjects();
        //lidarCont.Display(false);
    }

    IEnumerator MultipleScan()
    {
        float carSpeed = 50f / 36f;
        for (int i = 0; i < moveAndScanCount; i++)
        {
            lidarCont.Scan(false, i);
            if (i < moveAndScanCount - 1) egoCar.transform.position += Vector3.back * carSpeed;
            yield return new WaitForSeconds(0.1f);
        }
    }
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.X))
        {
            TurnOnOffRenderers(renderersTurnedOff);
            renderersTurnedOff = !renderersTurnedOff;
        }
        if (Input.GetKeyDown(KeyCode.S))
        {
            //lidarCont.Scan();
        }
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            processedShowed = !processedShowed;
            lidarCont.Display(processedShowed, 0);
        }
        if (Input.GetKeyDown(KeyCode.I))
        {
            displayedById = !displayedById;
            lidarCont.DisplayById(0, displayedById);
        }
        if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            lidarCont.Display(processedShowed, 1);
        }
        if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            lidarCont.Display(processedShowed, 2);
        }
        if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            linesShowed = !linesShowed;
            lidarCont.DisplayLines(linesShowed, false);
        }
        if (Input.GetKeyDown(KeyCode.Alpha5))
        {
            linesShowed = !linesShowed;
            lidarCont.DisplayLines(linesShowed, true);
        }
        if (Input.GetKeyDown(KeyCode.Alpha6))
        {
            objectsShowed = !objectsShowed;
            foreach (var obj in generatedModels) obj.SetActive(objectsShowed);
        }
        if (displayedById)
        {
            if (Input.GetKeyDown(KeyCode.LeftArrow)) lidarCont.DisplayById(-1, displayedById);
            if (Input.GetKeyDown(KeyCode.RightArrow)) lidarCont.DisplayById(1, displayedById);
            if (Input.GetKeyDown(KeyCode.UpArrow)) lidarCont.DisplayById(10, displayedById);
            if (Input.GetKeyDown(KeyCode.DownArrow)) lidarCont.DisplayById(-10, displayedById);
        }
    }

    void ImportObjects()
    {
        int counter = 0;
        GameObject obj = Resources.Load<GameObject>("Generated_Models_test/processed_obj_0");
        while (obj != null)
        {
            var instance = (Instantiate(obj));
            instance.AddComponent<MeshFilter>();
            instance.GetComponent<MeshFilter>().mesh = obj.transform.GetChild(0)
                    .GetComponent<MeshFilter>().sharedMesh;
            instance.AddComponent<MeshRenderer>();
            Material m = Instantiate(objMaterial);
            m.color = pointColors[(counter + 1) % pointColors.Length];
            instance.GetComponent<MeshRenderer>().material = m;
            generatedModels.Add(instance);
            instance.SetActive(false);
            counter++;
            obj = Resources.Load<GameObject>("Generated_Models_test/processed_obj_" + counter);
        }
    }

    void TurnOnOffRenderers(bool on)
    {
        if(renderers == null)
            renderers = FindObjectsOfType<MeshRenderer>();
        RenderSettings.skybox = on ? defaultSkybox : pointCloudSkybox;
        foreach (MeshRenderer m in renderers) m.enabled = on;
        foreach (SkinnedMeshRenderer m in humanRenderers) m.enabled = on;
    }

    public Color[] GetColors()
    {
        return pointColors;
    }
}
