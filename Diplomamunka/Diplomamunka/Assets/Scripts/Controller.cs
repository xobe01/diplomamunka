using System.Collections;
using System.Collections.Generic;
using System.Linq;
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
    [SerializeField] int generatedModelsIndex;
    [SerializeField] int visualizeFrameCount;

    Material defaultSkybox;
    LidarController lidarCont;
    List<MeshRenderer> renderers;
    SkinnedMeshRenderer[] humanRenderers;
    TrainDataGenerator traindataGen;
    List<List<GameObject>> generatedModels;
    bool renderersTurnedOff;
    bool processedShowed;
    bool displayedById;
    bool linesShowed;
    bool objectsShowed;    
    const float carSpeed = 50f / 36f;

    void Start()
    {
        traindataGen = FindObjectOfType<TrainDataGenerator>();
        generatedModels = new List<List<GameObject>>();
        defaultSkybox = RenderSettings.skybox;
        lidarCont = FindObjectOfType<LidarController>();
        humanRenderers = FindObjectsOfType<SkinnedMeshRenderer>();
        renderers = new List<MeshRenderer>(FindObjectsOfType<MeshRenderer>());
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
        if (visualizeFrameCount > 0)
            StartCoroutine(VisualizeModels());
        //lidarCont.Display(false);
    }

    IEnumerator VisualizeModels()
    {
        foreach(MeshRenderer r in egoCar.GetComponentsInChildren<MeshRenderer>())
        {
            renderers.Remove(r);
        }
        while (true) 
        {
            egoCar.transform.position = Vector3.zero;
            generatedModelsIndex = 0;
            foreach (var obj in generatedModels[generatedModelsIndex]) obj.SetActive(true);
            for (int i = 0; i < visualizeFrameCount - 1; i++)
            {
                yield return new WaitForSeconds(0.1f);
                foreach (var obj in generatedModels[generatedModelsIndex]) obj.SetActive(false);
                generatedModelsIndex++;
                foreach (var obj in generatedModels[generatedModelsIndex]) obj.SetActive(true);
                egoCar.transform.position += Vector3.back * carSpeed;
            }
            yield return new WaitForSeconds(1);
        }
    }

    IEnumerator MultipleScan()
    {
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
            foreach (var obj in generatedModels[generatedModelsIndex]) obj.SetActive(objectsShowed);
        }
        if (Input.GetKeyDown(KeyCode.G))
        {
            foreach (var obj in generatedModels[generatedModelsIndex]) obj.SetActive(false);
            generatedModelsIndex++;
            if (generatedModelsIndex > generatedModels.Count - 1) generatedModelsIndex = 0;
            if (objectsShowed)
                foreach (var obj in generatedModels[generatedModelsIndex]) obj.SetActive(true);
        }
        if (Input.GetKeyDown(KeyCode.F))
        {
            foreach (var obj in generatedModels[generatedModelsIndex]) obj.SetActive(false);
            generatedModelsIndex--;
            if (generatedModelsIndex < 0) generatedModelsIndex = generatedModels.Count - 1;
            if (objectsShowed)
                foreach (var obj in generatedModels[generatedModelsIndex]) obj.SetActive(true);
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
        for (int i = 0; i < 100; i++)
        {
            int counter = 0;
            GameObject obj = Resources.Load<GameObject>("Generated_Models_" + i + "/processed_obj_0");
            generatedModels.Add(new List<GameObject>());
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
                generatedModels[i].Add(instance);
                instance.SetActive(false);
                counter++;
                obj = Resources.Load<GameObject>("Generated_Models_" + i + "/processed_obj_" + counter);
            }
        }
    }

    void TurnOnOffRenderers(bool on)
    {
        if(renderers == null)
            renderers = new List<MeshRenderer>(FindObjectsOfType<MeshRenderer>());
        RenderSettings.skybox = on ? defaultSkybox : pointCloudSkybox;
        foreach (MeshRenderer m in renderers) m.enabled = on;
        foreach (SkinnedMeshRenderer m in humanRenderers) m.enabled = on;
    }

    public Color[] GetColors()
    {
        return pointColors;
    }
}
